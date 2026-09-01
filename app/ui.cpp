#include "ff_gui.h"
#include <algorithm>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>

namespace fs = std::filesystem;
namespace ff_gui {
namespace {
bool inputString(const char *label, std::string &value, std::size_t capacity = 256) {
  std::vector<char> buffer(std::max(capacity, value.size() + 1), '\0');
  std::copy(value.begin(), value.end(), buffer.begin());
  if (!ImGui::InputText(label, buffer.data(), buffer.size()))
    return false;
  value = buffer.data();
  return true;
}
template <typename T> bool eraseAt(std::vector<T> &items, int index) {
  if (index < 0 || static_cast<size_t>(index) >= items.size()) return false;
  items.erase(items.begin() + index); return true;
}
void tooltip(const char *text) {
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort |
                           ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 36.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}
std::string automaticTrainingFileName(const ff::TrainingSession &session) {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm localTime{};
  localtime_s(&localTime, &time);
  std::vector<int> assignedRods;
  for (const auto &aim : session.aims)
    if (aim.assigned) assignedRods.push_back(int(aim.address.rod));
  std::sort(assignedRods.begin(), assignedRods.end());
  assignedRods.erase(std::unique(assignedRods.begin(), assignedRods.end()),
                     assignedRods.end());
  std::ostringstream assignedRange;
  if (assignedRods.empty()) {
    assignedRange << "Unassigned";
  } else {
    for (size_t begin = 0; begin < assignedRods.size();) {
      size_t end = begin;
      while (end + 1 < assignedRods.size() &&
             assignedRods[end + 1] == assignedRods[end] + 1)
        ++end;
      if (begin) assignedRange << "_";
      assignedRange << "R" << assignedRods[begin];
      if (end > begin) assignedRange << "-" << assignedRods[end];
      begin = end + 1;
    }
  }
  std::ostringstream name;
  name << "Training_CH" << (static_cast<int>(session.channel) + 1)
       << "_Occ" << session.candidatesMm.size()
       << "_" << assignedRange.str() << "_"
       << std::put_time(&localTime, "%Y%m%d_%H%M%S")
       << ".json";
  return name.str();
}
} // namespace
static ff::FFConfig defaultConfig() {
  ff::FFConfig c;
  c.machineId = "FF-001";
  c.configVersion = "FF-CONFIG-1.1";
  c.maxActiveRodCount = 40;
  const int rodCounts[3] = {14, 14, 12};
  for (int i = 0; i < 3; i++) {
    auto &ch = c.channels[i];
    ch.id = static_cast<ff::ChannelId>(i);
    ch.controller.name = "CTRL-CH" + std::to_string(i + 1);
    ch.controller.serial.portName = "COM" + std::to_string(i + 3);
    for (int rodIndex = 0; rodIndex < rodCounts[i]; ++rodIndex) {
      ff::RodConfig r;
      r.id = static_cast<ff::RodId>(rodIndex);
      r.name = "CH" + std::to_string(i + 1) + "-R" + std::to_string(rodIndex);
      r.hardMin = 0;
      r.hardMax = 4095;
      r.softMin = 100;
      r.softMax = 4000;
      r.zeroPosition = 2048;
      r.safePosition = 3900;
      r.safePositionValid = true;
      // optical_position = P0 + P1*u.
      r.p0 = 0.0;
      r.p1 = 90.0 / 4095.0;
      r.maxVelocity = 300;
      r.maxAcceleration = 100;
      r.maxDeceleration = 100;
      r.positionTolerance = 5;
      ch.rods.push_back(r);
    }
  }
  return c;
}

MainUi::MainUi() {
  auto root =
      fs::path(std::getenv("PROGRAMDATA") ? std::getenv("PROGRAMDATA") : ".") /
      "RSIC" / "FF";
  fs::create_directories(root);
  const fs::path developmentConfig =
      R"(D:\RSIC\FF\config\defaults\ff_config.json)";
  // Development runs use the repository configuration explicitly requested by
  // the team. Installed builds fall back to ProgramData.
  configPath_ = (fs::exists(developmentConfig) ? developmentConfig
                                               : root / "ff_config.json")
                    .string();
  iniPath_ = (root / "ff_gui.ini").string();
  presetDirectory_ = (root / "position_presets").string();
  fs::create_directories(presetDirectory_);
  trainingDirectory_ = (root / "training_sessions").string();
  fs::create_directories(trainingDirectory_);
  for (int channel = 0; channel < 3; ++channel)
    trainingSessions_[channel].channel = static_cast<ff::ChannelId>(channel);
  training_ = trainingSessions_[0];
  refreshPresetSlots();
  loadUiSettings();
  scanSerialPorts();
  ensureConfig();
}

void MainUi::refreshPresetSlots() {
  presetSlots_.clear();
  // JSON is the canonical slot format.  Continue discovering legacy CSV
  // files so existing installations can load and migrate them by saving the
  // same slot again.
  const std::regex fileNamePattern(R"(^slot_([0-9]+)\.(?:json|csv)$)",
                                   std::regex::icase);
  std::error_code error;
  for (const auto &entry : fs::directory_iterator(presetDirectory_, error)) {
    if (error || !entry.is_regular_file())
      continue;
    std::smatch match;
    const auto name = entry.path().filename().string();
    if (!std::regex_match(name, match, fileNamePattern))
      continue;
    try {
      presetSlots_.push_back(std::stoi(match[1].str()));
    } catch (const std::exception &) {
      // Ignore file names outside the supported slot-number range.
    }
  }
  std::sort(presetSlots_.begin(), presetSlots_.end());
  presetSlots_.erase(std::unique(presetSlots_.begin(), presetSlots_.end()),
                     presetSlots_.end());
}

bool MainUi::hasPresetSlot(int slot) const {
  return std::find(presetSlots_.begin(), presetSlots_.end(), slot) !=
         presetSlots_.end();
}

std::string MainUi::presetPath(int slot) const {
  return (fs::path(presetDirectory_) / ("slot_" + std::to_string(slot) +
                                        ".json"))
      .string();
}

bool MainUi::savePreset(int slot,
                        const std::vector<ff::RodTarget> &targets) {
  ff::TrainingSession session;
  session.source = ff::TrainingSource::Loaded;
  session.channel = ff::ChannelId::CH1;
  session.complete = true;
  session.createdUnixSeconds = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  session.occlusionSummary =
      slot == 0 ? "Main UI Slot Initial" : "Main UI Slot" + std::to_string(slot);
  session.executionTargets = targets;
  if (const auto config = service_.config()) {
    session.machineId = config->machineId;
    session.configVersion = config->configVersion;
  }
  return static_cast<bool>(
      ff::TrainingAnalysis::saveSession(presetPath(slot), session));
}

std::vector<ff::RodTarget> MainUi::loadPreset(int slot) const {
  // Prefer the unified JSON format.  A legacy CSV with the same slot number
  // remains a read-only fallback until the operator saves that slot again.
  if (fs::exists(presetPath(slot))) {
    const auto session = ff::TrainingAnalysis::loadSession(presetPath(slot));
    if (session && !session.value.executionTargets.empty())
      return session.value.executionTargets;
    return {};
  }
  const auto legacyPath =
      (fs::path(presetDirectory_) /
       ("slot_" + std::to_string(slot) + ".csv")).string();
  std::ifstream input(legacyPath);
  std::vector<ff::RodTarget> targets;
  std::string line;
  std::getline(input, line); // CSV header
  while (std::getline(input, line)) {
    std::istringstream row(line);
    std::string channelText, rodText, positionText, velocityText;
    if (!std::getline(row, channelText, ',') || !std::getline(row, rodText, ',') ||
        !std::getline(row, positionText, ',') || !std::getline(row, velocityText))
      return {};
    try {
      const auto channel = std::stoi(channelText);
      const auto rod = std::stoi(rodText);
      const auto position = std::stoi(positionText);
      const auto velocity = std::stod(velocityText);
      if (channel < 0 || channel > 2 || rod < 0 || rod > 255)
        return {};
      targets.push_back({{static_cast<ff::ChannelId>(channel),
                          static_cast<ff::RodId>(rod)},
                         position, velocity});
    } catch (const std::exception &) {
      return {};
    }
  }
  return targets;
}

void MainUi::loadUiSettings() {
  std::ifstream input(iniPath_);
  std::string line;
  bool perChannelTrainingSettings = false;
  while (std::getline(input, line)) {
    const auto split = line.find('=');
    if (split == std::string::npos)
      continue;
    const auto key = line.substr(0, split);
    const auto value = line.substr(split + 1);
    if (key == "mode")
      simulation_ = value != "real";
    else if (key == "config_path" && !value.empty() && fs::exists(value))
      configPath_ = value;
    else if (key == "enabled_channels") {
      try {
        enabledChannelCount_ = std::clamp(std::stoi(value), 1, 3);
      } catch (const std::exception &) {
        // Ignore malformed user settings and preserve the safe default.
      }
    } else if (key.rfind("serial_ch", 0) == 0) {
      try {
        const int channel = std::stoi(key.substr(9));
        if (channel >= 0 && channel < 3)
          serialPorts_[channel] = value;
      } catch (const std::exception &) {
        // Ignore malformed user settings and preserve the safe defaults.
      }
    } else if (key.rfind("training_ch", 0) == 0 && key.size() > 13 &&
               key[11] >= '0' && key[11] <= '2' && key[12] == '_') {
      auto &session = trainingSessions_[key[11] - '0'];
      const auto field = key.substr(13);
      perChannelTrainingSettings = true;
      try {
        if (field == "source") {
          const int source = std::clamp(std::stoi(value), 0, 3);
          session.source = source == static_cast<int>(ff::TrainingSource::Loaded)
                               ? ff::TrainingSource::Sim1
                               : static_cast<ff::TrainingSource>(source);
        } else if (field == "start_mm") session.startMm = std::stod(value);
        else if (field == "end_mm") session.endMm = std::stod(value);
        else if (field == "step_mm") session.stepMm = std::stod(value);
        else if (field == "smoothing") session.valleySettings.smoothingPoints = std::stoi(value);
        else if (field == "prominence") session.valleySettings.minimumProminenceFraction = std::stod(value);
        else if (field == "distance_mm") session.valleySettings.minimumDistanceMm = std::stod(value);
        else if (field == "max_valleys") session.valleySettings.maximumCount = std::stoi(value);
      } catch (const std::exception &) {}
    } else if (key == "training_source") {
      try {
        const int source = std::clamp(std::stoi(value), 0, 3);
        // A loaded session has no durable path in the INI.  Start from Sim1
        // instead and let the user explicitly load that session again.
        training_.source = source == static_cast<int>(ff::TrainingSource::Loaded)
                               ? ff::TrainingSource::Sim1
                               : static_cast<ff::TrainingSource>(source);
      } catch (const std::exception &) {}
    } else if (key == "training_channel") {
      try { training_.channel = static_cast<ff::ChannelId>(std::clamp(std::stoi(value), 0, 2)); }
      catch (const std::exception &) {}
    } else if (key == "training_start_mm") {
      try { training_.startMm = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_end_mm") {
      try { training_.endMm = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_step_mm") {
      try { training_.stepMm = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_smoothing") {
      try { training_.valleySettings.smoothingPoints = std::stoi(value); } catch (const std::exception &) {}
    } else if (key == "training_prominence") {
      try { training_.valleySettings.minimumProminenceFraction = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_distance_mm") {
      try { training_.valleySettings.minimumDistanceMm = std::stod(value); } catch (const std::exception &) {}
    } else if (key == "training_max_valleys") {
      try { training_.valleySettings.maximumCount = std::stoi(value); } catch (const std::exception &) {}
    }
  }
  const auto normalizeTraining = [](ff::TrainingSession &session) {
    if (!std::isfinite(session.startMm)) session.startMm = 0.0;
    if (!std::isfinite(session.endMm)) session.endMm = 0.0;
    if (!std::isfinite(session.stepMm) || session.stepMm <= 0) session.stepMm = 0.1;
    session.valleySettings.smoothingPoints = std::max(1, session.valleySettings.smoothingPoints);
    if (!std::isfinite(session.valleySettings.minimumProminenceFraction) ||
        session.valleySettings.minimumProminenceFraction < 0)
      session.valleySettings.minimumProminenceFraction = 0.10;
    if (!std::isfinite(session.valleySettings.minimumDistanceMm) ||
        session.valleySettings.minimumDistanceMm < 0)
      session.valleySettings.minimumDistanceMm = 0.5;
    session.valleySettings.maximumCount = std::max(0, session.valleySettings.maximumCount);
  };
  if (!perChannelTrainingSettings) {
    normalizeTraining(training_);
    const int selected = std::clamp(static_cast<int>(training_.channel), 0, 2);
    for (int channel = 0; channel < 3; ++channel) {
      trainingSessions_[channel] = training_;
      trainingSessions_[channel].channel = static_cast<ff::ChannelId>(channel);
    }
    training_ = trainingSessions_[selected];
  } else {
    for (int channel = 0; channel < 3; ++channel) {
      trainingSessions_[channel].channel = static_cast<ff::ChannelId>(channel);
      normalizeTraining(trainingSessions_[channel]);
    }
    training_ = trainingSessions_[std::clamp(static_cast<int>(training_.channel), 0, 2)];
  }
}

void MainUi::saveUiSettings() const {
  std::ofstream output(iniPath_, std::ios::trunc);
  if (!output)
    return;
  output << "mode=" << (simulation_ ? "simulation" : "real") << "\n";
  output << "enabled_channels=" << enabledChannelCount_ << "\n";
  output << "config_path=" << configPath_ << "\n";
  for (int i = 0; i < 3; ++i)
    output << "serial_ch" << i << "=" << serialPorts_[i] << "\n";
  output << std::setprecision(17);
  auto sessions = trainingSessions_;
  sessions[std::clamp(static_cast<int>(training_.channel), 0, 2)] = training_;
  output << "training_source=" << static_cast<int>(training_.source) << "\n";
  output << "training_channel=" << static_cast<int>(training_.channel) << "\n";
  output << "training_start_mm=" << training_.startMm << "\n";
  output << "training_end_mm=" << training_.endMm << "\n";
  output << "training_step_mm=" << training_.stepMm << "\n";
  output << "training_smoothing=" << training_.valleySettings.smoothingPoints << "\n";
  output << "training_prominence="
         << training_.valleySettings.minimumProminenceFraction << "\n";
  output << "training_distance_mm="
         << training_.valleySettings.minimumDistanceMm << "\n";
  output << "training_max_valleys="
         << training_.valleySettings.maximumCount << "\n";
  for (int channel = 0; channel < 3; ++channel) {
    const auto &session = sessions[channel];
    const std::string key = "training_ch" + std::to_string(channel) + "_";
    output << key << "source=" << static_cast<int>(session.source) << "\n";
    output << key << "start_mm=" << session.startMm << "\n";
    output << key << "end_mm=" << session.endMm << "\n";
    output << key << "step_mm=" << session.stepMm << "\n";
    output << key << "smoothing=" << session.valleySettings.smoothingPoints << "\n";
    output << key << "prominence=" << session.valleySettings.minimumProminenceFraction << "\n";
    output << key << "distance_mm=" << session.valleySettings.minimumDistanceMm << "\n";
    output << key << "max_valleys=" << session.valleySettings.maximumCount << "\n";
  }
}

void MainUi::scanSerialPorts() {
  availablePorts_.clear();
  wchar_t deviceName[256]{};
  for (int port = 1; port <= 256; ++port) {
    const auto portName = L"COM" + std::to_wstring(port);
    const auto size =
        static_cast<DWORD>(sizeof(deviceName) / sizeof(deviceName[0]));
    if (QueryDosDeviceW(portName.c_str(), deviceName, size) != 0)
      availablePorts_.push_back("COM" + std::to_string(port));
  }
}

void MainUi::ensureConfig() {
  bool needsMigration = !fs::exists(configPath_);
  if (!needsMigration) {
    std::ifstream input(configPath_);
    std::string contents((std::istreambuf_iterator<char>(input)), {});
    needsMigration = contents.find("FF-CONFIG-1.0") != std::string::npos;
    if (!needsMigration)
      needsMigration = !ff::FFConfigIO::load(configPath_);
  }
  if (needsMigration)
    ff::FFConfigIO::save(configPath_, defaultConfig());
  auto x = service_.loadConfig(configPath_, simulation_, serialPorts_,
                               enabledChannelCount_);
  configLoaded_ = static_cast<bool>(x);
  targetsInitialized_ = false;
  if (!x)
    notice_ = x.message;
}
void MainUi::draw() {
  if (!configLoaded_ || !service_.connected()) {
    welcome();
  } else {
    dashboard();
  }
  if (showParameters_)
    parameters();
  if (showTraining_)
    training();
}
void MainUi::welcome() {
  static const char *modes[] = {"Simulation", "Real Hardware"};
  static const char *channelCounts[] = {"1", "2", "3"};
  constexpr float controlWidth = 240.0f;
  if (ImGui::BeginMenuBar()) {
    if (ImGui::BeginMenu("Tools")) {
      if (ImGui::MenuItem("FF_Config")) {
        showParameters_ = true;
        initializeParameters();
      }
      if (ImGui::MenuItem("Training")) showTraining_ = true;
      ImGui::EndMenu();
    }
    ImGui::EndMenuBar();
  }
  int mode = simulation_ ? 0 : 1;
  ImGui::Text("FF Connection Center");
  ImGui::Separator();
  ImGui::SetNextItemWidth(controlWidth);
  if (ImGui::Combo("Mode", &mode, modes, IM_ARRAYSIZE(modes))) {
    simulation_ = mode == 0;
    saveUiSettings();
  }
  int channelCountIndex = enabledChannelCount_ - 1;
  ImGui::SetNextItemWidth(controlWidth);
  if (ImGui::Combo("Enabled Channels", &channelCountIndex, channelCounts,
                   IM_ARRAYSIZE(channelCounts))) {
    enabledChannelCount_ = channelCountIndex + 1;
    if (activeChannel_ >= enabledChannelCount_)
      activeChannel_ = 0;
    saveUiSettings();
  }
  ImGui::SameLine();
  if (ImGui::Button("Scan Serial Ports"))
    scanSerialPorts();
  ImGui::TextDisabled("%zu serial port(s) detected", availablePorts_.size());
  for (int i = 0; i < enabledChannelCount_; ++i) {
    const auto label = "CH" + std::to_string(i + 1) + " Serial Port";
    const char *preview =
        serialPorts_[i].empty() ? "Select a port" : serialPorts_[i].c_str();
    ImGui::PushID(i);
    ImGui::BeginDisabled(simulation_ || availablePorts_.empty());
    ImGui::SetNextItemWidth(controlWidth);
    if (ImGui::BeginCombo(label.c_str(), preview)) {
      for (const auto &port : availablePorts_) {
        const bool selected = serialPorts_[i] == port;
        if (ImGui::Selectable(port.c_str(), selected)) {
          serialPorts_[i] = port;
          saveUiSettings();
        }
        if (selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    ImGui::PopID();
  }
  ImGui::Separator();
  ImGui::BeginDisabled(showParameters_);
  if (ImGui::Button("Open FF", ImVec2(180, 0))) {
    bool duplicatePorts = false;
    for (int i = 0; i < enabledChannelCount_; ++i)
      for (int j = i + 1; j < enabledChannelCount_; ++j)
        duplicatePorts = duplicatePorts || serialPorts_[i] == serialPorts_[j];
    const bool missingPort = std::any_of(
        serialPorts_.begin(), serialPorts_.begin() + enabledChannelCount_,
        [this](const auto &port) {
          return std::find(availablePorts_.begin(), availablePorts_.end(),
                           port) == availablePorts_.end();
        });
    if (!simulation_ && (duplicatePorts || missingPort)) {
      notice_ =
          "Select different detected COM ports for every enabled channel.";
    } else {
      auto loaded = service_.loadConfig(configPath_, simulation_, serialPorts_,
                                        enabledChannelCount_);
      if (!loaded) {
        configLoaded_ = false;
        notice_ = loaded.message;
      } else {
        configLoaded_ = true;
        auto x = service_.connect();
        if (!x)
          notice_ = x.message;
        else
          notice_.clear();
      }
    }
  }
  ImGui::EndDisabled();
  if (!notice_.empty())
    ImGui::TextColored(ImVec4(1, 0.5f, 0.2f, 1), "%s", notice_.c_str());
}
void MainUi::dashboard() {
  if (ImGui::BeginMenuBar()) {
    if (ImGui::BeginMenu("Tools")) {
      if (ImGui::MenuItem("FF_Config")) { showParameters_ = true; initializeParameters(); }
      if (ImGui::MenuItem("Training")) showTraining_ = true;
      ImGui::EndMenu();
    }
    ImGui::EndMenuBar();
  }
  auto s = service_.status();
  ImGui::Text("RS485 is connected.  Mode: %s",
              simulation_ ? "Simulation" : "Real Hardware");
  ImGui::SameLine();
  if (ImGui::Button("close")) {
    auto x = service_.disconnect();
    if (!x)
      notice_ = x.message;
    targetsInitialized_ = false;
    return;
  }
  ImGui::SameLine();
  if (ImGui::Button("EMERGENCY STOP"))
    service_.emergencyStop();

  if (ImGui::BeginTabBar("channels")) {
    for (int i = 0; i < service_.activeChannelCount(); i++) {
      const std::string label = "CH" + std::to_string(i + 1);
      if (ImGui::BeginTabItem(label.c_str())) {
        activeChannel_ = i;
        drawChannel(i, s);
        ImGui::EndTabItem();
      }
    }
    ImGui::EndTabBar();
  }
  ImGui::SeparatorText("Control");
  if (!targetsInitialized_ && service_.connected()) {
    targets_.clear();
    for (int i = 0; i < service_.activeChannelCount(); i++) {
      const auto &ch = s.channels[i];
      for (size_t rodIndex = 0; rodIndex < ch.rods.size(); rodIndex++)
        targets_.push_back(
            {{static_cast<ff::ChannelId>(i), static_cast<ff::RodId>(rodIndex)},
             ch.rods[rodIndex].actualPosition,
             100});
    }
    targetsInitialized_ = true;
  }

  constexpr int filePresetSlot = -1;
  int highestSlot = 0;
  if (!presetSlots_.empty())
    highestSlot = std::max(highestSlot, presetSlots_.back());
  const int newSlot = std::max(1, highestSlot + 1);
  const auto slotLabel = [&](int slot) {
    if (slot == filePresetSlot) return std::string("Slot (file)");
    if (slot == 0) return std::string("Slot Initial");
    if (slot == newSlot && !hasPresetSlot(slot))
      return std::string("Slot (new)");
    return "Slot" + std::to_string(slot);
  };
  const auto selectedSlotLabel = slotLabel(selectedPresetSlot_);
  ImGui::SetNextItemWidth(120.0f);
  if (ImGui::BeginCombo("Position Slot", selectedSlotLabel.c_str())) {
    if (ImGui::Selectable("Slot Initial", selectedPresetSlot_ == 0))
      selectedPresetSlot_ = 0;
    for (int slot : presetSlots_) {
      if (slot == 0) continue;
      const auto label = slotLabel(slot);
      if (ImGui::Selectable(label.c_str(), selectedPresetSlot_ == slot))
        selectedPresetSlot_ = slot;
    }
    if (ImGui::Selectable("Slot (new)", selectedPresetSlot_ == newSlot))
      selectedPresetSlot_ = newSlot;
    if (ImGui::Selectable("Slot (file)",
                          selectedPresetSlot_ == filePresetSlot))
      selectedPresetSlot_ = filePresetSlot;
    ImGui::EndCombo();
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(selectedPresetSlot_ == filePresetSlot);
  if (ImGui::Button("save")) {
    auto refreshed = service_.refresh();
    auto current = service_.status();
    bool safe = static_cast<bool>(refreshed);
    for (int i = 0; i < service_.activeChannelCount(); ++i)
      safe = safe && current.safety[i].state == ff::SafetyState::Safe;
    if (!safe) {
      notice_ = refreshed ? "Current positions are unsafe; preset was not saved."
                          : refreshed.message;
    } else {
      std::vector<ff::RodTarget> preset;
      for (int i = 0; i < service_.activeChannelCount(); ++i) {
        const auto &channel = current.channels[i];
        for (size_t rod = 0; rod < channel.rods.size(); ++rod)
          preset.push_back({{static_cast<ff::ChannelId>(i),
                             static_cast<ff::RodId>(rod)},
                            channel.rods[rod].actualPosition, 100.0});
      }
      if (savePreset(selectedPresetSlot_, preset)) {
        targets_ = std::move(preset);
        refreshPresetSlots();
        notice_ = "Positions saved to slot " +
                  std::to_string(selectedPresetSlot_) + ".";
      } else {
        notice_ = "Could not write the position preset file.";
      }
    }
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(selectedPresetSlot_ != filePresetSlot &&
                       !hasPresetSlot(selectedPresetSlot_));
  if (ImGui::Button("load")) {
    if (selectedPresetSlot_ == filePresetSlot) {
      IGFD::FileDialogConfig dialogConfig;
      dialogConfig.path = trainingDirectory_;
      ImGuiFileDialog::Instance()->OpenDialog(
          "MainTrainingSlotLoad", "Load Training Position File",
          ".json", dialogConfig);
    } else {
      auto preset = loadPreset(selectedPresetSlot_);
      if (preset.empty()) {
        notice_ = "The selected position preset is invalid or empty.";
      } else {
        auto x = service_.setRodTargets(preset);
        if (x) {
          targets_ = std::move(preset);
          notice_ = slotLabel(selectedPresetSlot_) + " loaded.";
        } else {
          notice_ = x.message + "; restored to the current safe positions.";
        }
      }
    }
  }
  ImGui::EndDisabled();
  if (ImGuiFileDialog::Instance()->Display("MainTrainingSlotLoad")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      const auto path = ImGuiFileDialog::Instance()->GetFilePathName();
      auto loaded = ff::TrainingAnalysis::loadSession(path);
      if (!loaded) {
        notice_ = "Training position file rejected: " + loaded.message;
      } else if (const auto runtimeConfig = service_.config();
                 !runtimeConfig ||
                 loaded.value.machineId != runtimeConfig->machineId ||
                 loaded.value.configVersion != runtimeConfig->configVersion) {
        notice_ = "Training position file rejected: machine/config does not match.";
      } else if (loaded.value.executionTargets.empty()) {
        notice_ = "Training position file has no executable target set; run Train and save it again.";
      } else {
        const auto runtimeConfig = service_.config();
        bool completeTargets = static_cast<bool>(runtimeConfig);
        std::array<std::set<int>, 3> fileRods;
        std::array<bool, 3> includedChannels{};
        if (completeTargets) {
          for (const auto &target : loaded.value.executionTargets) {
            const int channel = static_cast<int>(target.address.channel);
            if (channel < 0 || channel >= service_.activeChannelCount() ||
                !fileRods[channel].insert(int(target.address.rod)).second) {
              completeTargets = false;
              break;
            }
            includedChannels[channel] = true;
          }
        }
        bool hasCompleteChannel = false;
        for (int channel = 0;
             completeTargets && channel < service_.activeChannelCount();
             ++channel) {
          if (!includedChannels[channel]) continue;
          hasCompleteChannel = true;
          const auto &configuredRods = runtimeConfig->channels[channel].rods;
          completeTargets = fileRods[channel].size() == configuredRods.size();
          for (const auto &rod : configuredRods)
            completeTargets = completeTargets &&
                              fileRods[channel].count(int(rod.id)) == 1;
        }
        completeTargets = completeTargets && hasCompleteChannel;
        if (!completeTargets) {
          notice_ = "Position JSON rejected: every included channel must contain all current Rod targets exactly once.";
        } else {
          auto moved = service_.setRodTargets(loaded.value.executionTargets);
          if (!moved) {
            notice_ = "Training position file rejected by motion safety: " +
                      moved.message;
          } else {
            for (const auto &fileTarget : loaded.value.executionTargets) {
              const auto mainTarget = std::find_if(
                  targets_.begin(), targets_.end(),
                  [&](const ff::RodTarget &entry) {
                    return entry.address == fileTarget.address;
                  });
              if (mainTarget == targets_.end())
                targets_.push_back(fileTarget);
              else
                *mainTarget = fileTarget;
            }
            notice_ = "Training position file loaded and executed";
            if (!loaded.value.occlusionSummary.empty())
              notice_ += ": " + loaded.value.occlusionSummary;
            else
              notice_ += ".";
          }
        }
      }
    }
    ImGuiFileDialog::Instance()->Close();
  }

  for (auto &r : targets_) {
    if (static_cast<int>(r.address.channel) != activeChannel_)
      continue;
    const auto config = service_.rodConfig(r.address);
    if (!config || std::abs(config->p1) < 1e-12)
      continue;

    const auto toOpticalPosition = [&config](int position) {
      return config->p0 + position * config->p1;
    };
    const auto toEncoder = [&config](float opticalPosition) {
      return static_cast<int>(
          std::lround((opticalPosition - config->p0) / config->p1));
    };
    const float minimumMm =
        static_cast<float>(std::min(toOpticalPosition(config->softMin),
                                    toOpticalPosition(config->softMax)));
    const float maximumMm =
        static_cast<float>(std::max(toOpticalPosition(config->softMin),
                                    toOpticalPosition(config->softMax)));
    float targetMm = static_cast<float>(toOpticalPosition(r.position));

    ImGui::SetNextItemWidth(520.0f);
    if (ImGui::SliderFloat(("stroke" + std::to_string(r.address.rod) +
                            "##CH" +
                            std::to_string(int(r.address.channel) + 1)).c_str(),
                           &targetMm, minimumMm, maximumMm, "%.3f mm")) {
      const auto previousPosition = r.position;
      r.position =
          std::clamp(toEncoder(targetMm), config->softMin, config->softMax);
      auto x = service_.setRodTarget(r);
      if (!x) {
        r.position = previousPosition;
        targetMm = static_cast<float>(toOpticalPosition(previousPosition));
        notice_ = x.message + "; restored to the previous safe position.";
      } else {
        notice_.clear();
      }
    }
  }
  if (!notice_.empty())
    ImGui::TextColored(ImVec4(1, 0.5f, 0.2f, 1), "%s", notice_.c_str());
}

bool MainUi::candidateSetIsLegal(const std::vector<double> &candidates,
                                 std::string *reason) const {
  constexpr double candidateCentreClearanceMm = 3.0;
  const double left = std::min(training_.startMm, training_.endMm) + 1.5;
  const double right = std::max(training_.startMm, training_.endMm) - 1.5;
  for (double candidate : candidates) {
    if (candidate < left - 1e-9 || candidate > right + 1e-9) {
      if (reason) *reason = "A complete 3.0 mm Rod window must remain inside the scan range.";
      return false;
    }
  }
  for (size_t i = 0; i < candidates.size(); ++i) {
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (std::abs(candidates[i] - candidates[j]) + 1e-9 <
          candidateCentreClearanceMm) {
        if (reason)
          *reason = "Candidate centers must be at least 3.0 mm apart.";
        return false;
      }
    }
  }
  const auto config = service_.config();
  if (!config) {
    if (reason) *reason = "No FF configuration is loaded.";
    return false;
  }
  const auto result = ff::TrainingAnalysis::validateAssignment(
      *config, service_.status(), training_.channel, candidates,
      training_.trainingRod);
  if (!result && reason) *reason = result.message;
  return static_cast<bool>(result);
}

void MainUi::resetTrainingMeasurement() {
  // Keep the operator's per-channel settings, but discard every value that
  // belongs to the previous acquisition / editing / training cycle.
  training_.samples.clear();
  training_.candidatesMm.clear();
  training_.aims.clear();
  training_.rodStates.clear();
  training_.occlusionSummary.clear();
  training_.executionTargets.clear();
  training_.complete = false;
  training_.machineId.clear();
  training_.configVersion.clear();
  training_.createdUnixSeconds = 0;

  trainingExecutionTargets_.clear();
  trainingEvasionTarget_.reset();
  trainingScanning_ = false;
  trainingMovePending_ = false;
  trainingNextMm_ = training_.startMm;
  trainingStableSince_.reset();
  trainingDraggingCandidate_ = -1;
  trainingDragOriginalMm_ = 0.0;
  trainingDragLastSafeMm_ = 0.0;
  trainingForbiddenBands_.clear();
  trainingForbiddenDirty_ = true;

  // The channel cache must not restore the discarded measurement when the
  // operator switches away from this tab and back again.
  const int channel = static_cast<int>(training_.channel);
  if (channel >= 0 && channel < static_cast<int>(trainingSessions_.size()))
    trainingSessions_[channel] = training_;
}

void MainUi::refreshTrainingForbiddenBands() {
  if (!trainingForbiddenDirty_) return;
  trainingForbiddenBands_.clear();
  const auto config = service_.config();
  if (!config || static_cast<int>(training_.channel) >= static_cast<int>(config->channels.size())) {
    trainingForbiddenDirty_ = false;
    return;
  }

  const double left = std::min(training_.startMm, training_.endMm);
  const double right = std::max(training_.startMm, training_.endMm);
  if (right - left < 1e-9) {
    trainingForbiddenDirty_ = false;
    return;
  }

  // A Rod occupies 3 mm measured centre-to-centre.  Represent that occupied
  // space exactly (rather than sampling it on a coarse grid), which makes the
  // red bands immediate and keeps Find Valleys / line edits responsive.
  constexpr double candidateCentreClearanceMm = 3.0;
  constexpr double halfClearanceMm = candidateCentreClearanceMm / 2.0;
  for (double candidate : training_.candidatesMm) {
    const double begin = std::max(left, candidate - halfClearanceMm);
    const double end = std::min(right, candidate + halfClearanceMm);
    if (begin < end) trainingForbiddenBands_.emplace_back(begin, end);
  }
  // Reaching the allocation capacity means that a *new* candidate will be
  // rejected by candidateSetIsLegal().  It does not make the already placed
  // candidate positions illegal, so do not turn the entire plot into a red
  // band here.  The visible bands remain the 3 mm occupied windows only.
  std::sort(trainingForbiddenBands_.begin(), trainingForbiddenBands_.end());
  std::vector<std::pair<double, double>> merged;
  for (const auto &band : trainingForbiddenBands_) {
    if (merged.empty() || band.first > merged.back().second)
      merged.push_back(band);
    else
      merged.back().second = std::max(merged.back().second, band.second);
  }
  trainingForbiddenBands_ = std::move(merged);
  trainingForbiddenDirty_ = false;
}

bool MainUi::updateTrainingAssignment() {
  trainingExecutionTargets_.clear();
  trainingEvasionTarget_.reset();
  const auto config = service_.config();
  if (!config) { trainingNotice_ = "No FF configuration is loaded."; return false; }
  const int channelIndex = static_cast<int>(training_.channel);
  const auto &channelConfig = config->channels[channelIndex];
  const auto current = service_.status();
  const auto trainingRod = std::find_if(
      channelConfig.rods.begin(), channelConfig.rods.end(),
      [&](const ff::RodConfig &rod) { return rod.id == training_.trainingRod; });
  const auto membership = std::find_if(
      channelConfig.layerMembers.begin(), channelConfig.layerMembers.end(),
      [&](const ff::LayerMembership &member) {
        return member.layer == 2 && member.rod == training_.trainingRod;
      });
  if (trainingRod == channelConfig.rods.end() ||
      membership == channelConfig.layerMembers.end()) {
    trainingNotice_ = "Training Rod has no Layer 2 evasion neighbor.";
    return false;
  }
  const int neighborId = membership->rightNeighbor >= 0
      ? membership->rightNeighbor : membership->leftNeighbor;
  const auto neighbor = std::find_if(
      channelConfig.rods.begin(), channelConfig.rods.end(),
      [&](const ff::RodConfig &rod) { return int(rod.id) == neighborId; });
  if (neighbor == channelConfig.rods.end()) {
    trainingNotice_ = "Training Rod has no Layer 2 evasion neighbor.";
    return false;
  }
  const double minMm = trainingRod->p0 + trainingRod->p1 * trainingRod->softMin;
  const double maxMm = trainingRod->p0 + trainingRod->p1 * trainingRod->softMax;
  const bool neighborIsRight = membership->rightNeighbor == neighborId;
  const auto evasionEncoder = neighborIsRight
      ? (minMm >= maxMm ? trainingRod->softMin : trainingRod->softMax)
      : (minMm <= maxMm ? trainingRod->softMin : trainingRod->softMax);
  trainingEvasionTarget_ = ff::RodTarget{{training_.channel,
                                           training_.trainingRod},
                                          evasionEncoder,
                                          trainingRod->maxVelocity};

  // The final state moves the Training Rod to this evasion position.  Use
  // that final position while selecting the obstruction Rods, otherwise a
  // freshly initialized Training Rod can falsely block every assignment.
  auto assignmentStatus = current;
  const auto trainingRodIndex = static_cast<size_t>(
      std::distance(channelConfig.rods.begin(), trainingRod));
  if (trainingRodIndex < assignmentStatus.channels[channelIndex].rods.size()) {
    auto &status = assignmentStatus.channels[channelIndex].rods[trainingRodIndex];
    status.actualPosition = evasionEncoder;
    status.targetPosition = evasionEncoder;
    status.positionValid = true;
  }
  const auto assignment = ff::TrainingAnalysis::assign(
      *config, assignmentStatus, training_.channel, training_.candidatesMm,
      training_.trainingRod);
  if (!assignment) { training_.aims.clear(); trainingNotice_ = assignment.message; return false; }
  training_.aims = assignment.value.aims;
  const auto &channelStatus = current.channels[channelIndex];
  std::vector<ff::RodTarget> targets;
  for (size_t index = 0; index < channelConfig.rods.size(); ++index) {
    if (index >= channelStatus.rods.size() || !channelStatus.rods[index].positionValid) {
      trainingNotice_ = assignment.value.message +
          ". Execute unavailable: current Rod feedback is incomplete.";
      return true;
    }
    const auto &rod = channelConfig.rods[index];
    targets.push_back({{training_.channel, rod.id},
                       channelStatus.rods[index].actualPosition,
                       rod.maxVelocity});
  }
  const auto setTarget = [&](const ff::RodTarget &replacement) {
    const auto target = std::find_if(targets.begin(), targets.end(),
        [&](const ff::RodTarget &entry) { return entry.address == replacement.address; });
    if (target != targets.end()) *target = replacement;
  };
  setTarget(*trainingEvasionTarget_);
  for (const auto &aim : training_.aims) if (aim.assigned) {
    const auto rod = std::find_if(channelConfig.rods.begin(), channelConfig.rods.end(),
        [&](const ff::RodConfig &entry) { return entry.id == aim.address.rod; });
    if (rod != channelConfig.rods.end())
      setTarget({aim.address, aim.encoderAim, rod->maxVelocity});
  }
  training_.machineId = config->machineId;
  training_.configVersion = config->configVersion;
  const auto validated = service_.validateRodTargets(targets);
  if (validated) {
    trainingExecutionTargets_ = std::move(targets);
    trainingNotice_ = assignment.value.message +
        ". Complete channel target state is safe and ready to execute.";
  } else {
    trainingNotice_ = assignment.value.message +
        ". Execute unavailable: " + validated.message;
  }
  return true;
}

void MainUi::tickTrainingScan() {
  if (!trainingScanning_) return;
  const auto config = service_.config();
  if (!config) { trainingScanning_ = false; trainingNotice_ = "Scan stopped: configuration is unavailable."; return; }
  const int channel = static_cast<int>(training_.channel);
  auto &rods = config->channels[channel].rods;
  const auto it = std::find_if(rods.begin(), rods.end(), [&](const ff::RodConfig &r) { return r.id == training_.trainingRod; });
  if (it == rods.end() || std::abs(it->p1) < 1e-12) { trainingScanning_ = false; trainingNotice_ = "Scan stopped: training Rod calibration is invalid."; return; }
  const int rodIndex = static_cast<int>(std::distance(rods.begin(), it));
  // The simulation backend applies a goal synchronously.  Waiting the normal
  // hardware stable-time (100 ms by default) for every 0.1 mm point made a
  // 701-point simulation take over a minute.  Keep the hardware state machine
  // intact, but advance bounded synchronous simulation transitions per frame.
  const bool fastSimulation = service_.simulation() &&
      (training_.source == ff::TrainingSource::Sim1 || training_.source == ff::TrainingSource::Sim2);
  const int budget = fastSimulation ? 48 : 1;
  for (int transition = 0; transition < budget && trainingScanning_; ++transition) {
    if (trainingMovePending_) {
      auto refreshed = service_.refresh();
      const auto status = service_.status();
      if (!refreshed || rodIndex >= static_cast<int>(status.channels[channel].rods.size())) {
        trainingScanning_ = false; trainingMovePending_ = false;
        trainingNotice_ = refreshed ? "Scan stopped: Rod feedback is unavailable." : refreshed.message;
        break;
      }
      const auto &feedback = status.channels[channel].rods[rodIndex];
      const auto requestedEncoder = static_cast<int>(std::lround((trainingNextMm_ - it->p0) / it->p1));
      if (!feedback.positionValid || std::abs(feedback.actualPosition - requestedEncoder) > it->positionTolerance ||
          feedback.state == ff::RodState::Fault) { trainingStableSince_.reset(); break; }
      if (!fastSimulation) {
        if (!trainingStableSince_) { trainingStableSince_ = std::chrono::steady_clock::now(); break; }
        if (std::chrono::steady_clock::now() - *trainingStableSince_ < it->stableTime) break;
      }
      const double actualMm = it->p0 + it->p1 * feedback.actualPosition;
      training_.samples.push_back({actualMm, ff::TrainingAnalysis::simulatedEnergy(training_.source, actualMm, training_.startMm, training_.endMm)});
      const double direction = training_.endMm >= training_.startMm ? 1.0 : -1.0;
      trainingNextMm_ += direction * training_.stepMm;
      trainingMovePending_ = false;
      trainingStableSince_.reset();
      if ((direction > 0 && trainingNextMm_ > training_.endMm + 1e-9) ||
          (direction < 0 && trainingNextMm_ < training_.endMm - 1e-9)) {
        trainingScanning_ = false; training_.complete = true;
        trainingNotice_ = "Scan complete.";
      }
      continue;
    }
    const auto encoder = static_cast<int>(std::lround((trainingNextMm_ - it->p0) / it->p1));
    if (encoder < it->softMin || encoder > it->softMax) {
      trainingScanning_ = false; trainingNotice_ = "Scan stopped: next point is outside the soft range."; break;
    }
    ff::RodTarget moveTarget{{training_.channel, training_.trainingRod}, encoder, it->maxVelocity};
    auto move = service_.setRodTarget(moveTarget);
    if (!move) { trainingScanning_ = false; trainingNotice_ = "Scan stopped: " + move.message; break; }
    trainingMovePending_ = true;
  }
}

void MainUi::training() {
  constexpr float sourceWidth = 120.0f;
  constexpr float numberWidth = 108.0f;
  constexpr float actionWidth = 112.0f;
  bool open = showTraining_;
  bool trainingSettingsChanged = false;
  ImGui::SetNextWindowSize(ImVec2(1120, 760), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Training", &open)) { ImGui::End(); showTraining_ = open; return; }
  showTraining_ = open;
  if (!open && trainingScanning_) {
    service_.cancelMotion(); trainingScanning_ = false; trainingMovePending_ = false; trainingStableSince_.reset();
    training_.complete = false; trainingNotice_ = "Scan stopped when the window was closed; partial samples were retained.";
  }
  if (!service_.connected()) {
    ImGui::TextColored(ImVec4(1, .5f, .2f, 1), "Open FF before starting a training scan.");
    ImGui::End(); return;
  }
  tickTrainingScan();
  const auto config = service_.config();
  if (!config) { ImGui::Text("No configuration is loaded."); ImGui::End(); return; }
  int channel = static_cast<int>(training_.channel);
  ImGui::BeginDisabled(trainingScanning_);
  if (ImGui::BeginTabBar("TrainingChannels")) {
    for (int candidate = 0; candidate < service_.activeChannelCount(); ++candidate) {
      const std::string label = "CH" + std::to_string(candidate + 1);
      if (!ImGui::BeginTabItem(label.c_str())) continue;
      if (channel != candidate) {
      trainingSessions_[channel] = training_;
      channel = candidate;
      training_ = trainingSessions_[channel];
      training_.channel = static_cast<ff::ChannelId>(channel);
      trainingExecutionTargets_.clear();
      trainingEvasionTarget_.reset();
      trainingForbiddenDirty_ = true;
      trainingDraggingCandidate_ = -1;
      trainingNotice_.clear();
      trainingSettingsChanged = true;
      }
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::EndDisabled();
  const auto &rods = config->channels[channel].rods;
  const auto configuredTrainingRod = config->channels[channel].trainingRod;
  if (configuredTrainingRod)
    training_.trainingRod = *configuredTrainingRod;
  const auto layer2TrainingRodCount = static_cast<int>(std::count_if(
      rods.begin(), rods.end(), [&](const ff::RodConfig &rod) {
        bool inLayer2 = false, inOtherLayer = false;
        for (const auto &member : config->channels[channel].layerMembers) {
          if (member.rod != rod.id) continue;
          inLayer2 = inLayer2 || member.layer == 2;
          inOtherLayer = inOtherLayer || member.layer != 2;
        }
        return inLayer2 && !inOtherLayer && rod.id != training_.trainingRod;
      }));
  const bool trainingCandidateCapacityReached =
      layer2TrainingRodCount > 0 &&
      training_.candidatesMm.size() >=
          static_cast<size_t>(layer2TrainingRodCount);
  // Fresh installations used zero-initialized scan settings.  A zero-width
  // scan produces only one point, which a line plot cannot show.
  if (!trainingRangeInitialized_[channel] && !rods.empty()) {
    const auto selected = std::find_if(rods.begin(), rods.end(),
      [&](const ff::RodConfig &entry) { return entry.id == training_.trainingRod; });
    if (selected != rods.end() && std::abs(selected->p1) > 1e-12 &&
        std::abs(training_.startMm - training_.endMm) < 1e-9) {
      training_.startMm = std::min(selected->p0 + selected->p1 * selected->softMin,
                                   selected->p0 + selected->p1 * selected->softMax);
      training_.endMm = std::max(selected->p0 + selected->p1 * selected->softMin,
                                 selected->p0 + selected->p1 * selected->softMax);
      saveUiSettings();
    }
    trainingRangeInitialized_[channel] = true;
  }
  const auto currentStatus = service_.status();
  if (ImGui::CollapsingHeader("Settings")) {
  const char *sources[] = {"Sim1", "Sim2", "Real", "Loaded"};
  int source = static_cast<int>(training_.source);
  ImGui::SetNextItemWidth(sourceWidth);
  if (ImGui::Combo("Data Source", &source, sources, IM_ARRAYSIZE(sources))) {
    training_.source = static_cast<ff::TrainingSource>(source);
    training_.aims.clear();
    trainingExecutionTargets_.clear();
    trainingEvasionTarget_.reset();
    trainingSettingsChanged = true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("Real: Itek camera runtime/hardware is not available in this build.");
  if (configuredTrainingRod) {
    ImGui::TextDisabled("Training Rod: Rod %d (FF config)",
                        int(*configuredTrainingRod));
  } else {
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f),
                       "Training Rod: not configured in FF config.");
  }
  const auto selectedRod = configuredTrainingRod
      ? std::find_if(rods.begin(), rods.end(), [&](const ff::RodConfig &entry) {
          return entry.id == *configuredTrainingRod;
        })
      : rods.end();
  if (selectedRod != rods.end()) {
    const auto rodIndex = static_cast<size_t>(std::distance(rods.begin(), selectedRod));
    if (rodIndex < currentStatus.channels[channel].rods.size()) {
      const auto &feedback = currentStatus.channels[channel].rods[rodIndex];
      ImGui::SameLine();
      ImGui::TextDisabled("Current: %d (%.4f mm)  Target: %d (%.4f mm)",
                          feedback.actualPosition,
                          selectedRod->p0 + selectedRod->p1 * feedback.actualPosition,
                          feedback.targetPosition,
                          selectedRod->p0 + selectedRod->p1 * feedback.targetPosition);
    }
  }
  ImGui::SetNextItemWidth(numberWidth);
  trainingSettingsChanged |= ImGui::InputDouble("Start (mm)", &training_.startMm, .1, 1, "%.4f");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(numberWidth);
  trainingSettingsChanged |= ImGui::InputDouble("End (mm)", &training_.endMm, .1, 1, "%.4f");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(numberWidth);
  trainingSettingsChanged |= ImGui::InputDouble("Step (mm)", &training_.stepMm, .01, .1, "%.4f");
  ImGui::SeparatorText("Valley Detection");
  ImGui::SetNextItemWidth(numberWidth); trainingSettingsChanged |= ImGui::InputInt("Smoothing", &training_.valleySettings.smoothingPoints);
  ImGui::SameLine(); ImGui::SetNextItemWidth(numberWidth); trainingSettingsChanged |= ImGui::InputDouble("Prominence", &training_.valleySettings.minimumProminenceFraction, .01, .1, "%.3f");
  ImGui::SameLine(); ImGui::SetNextItemWidth(numberWidth); trainingSettingsChanged |= ImGui::InputDouble("Min Distance (mm)", &training_.valleySettings.minimumDistanceMm, .1, 1, "%.3f");
  ImGui::SameLine(); ImGui::SetNextItemWidth(numberWidth); trainingSettingsChanged |= ImGui::InputInt("Max Valleys", &training_.valleySettings.maximumCount);
  }
  if (trainingSettingsChanged) trainingForbiddenDirty_ = true;
  const bool canScan = configuredTrainingRod && !trainingScanning_ && training_.source != ff::TrainingSource::Real &&
                       training_.source != ff::TrainingSource::Loaded;
  ImGui::BeginDisabled(trainingScanning_);
  if (ImGui::Button("Initialize", ImVec2(actionWidth, 0))) {
    resetTrainingMeasurement();
    if (!hasPresetSlot(0)) {
      trainingNotice_ = "Initialize unavailable: Position Slot 0 has not been saved.";
    } else {
      auto initialTargets = loadPreset(0);
      if (initialTargets.empty()) {
        trainingNotice_ = "Initialize failed: Position Slot 0 is invalid or empty.";
      } else {
        auto initialized = service_.setRodTargets(initialTargets);
        if (initialized) {
          service_.refresh();
          targets_ = std::move(initialTargets);
          trainingNotice_ = "Moved to the positions saved in Position Slot 0.";
        } else {
          trainingNotice_ = "Initialize failed: " + initialized.message;
        }
      }
    }
  }
  tooltip("Move all Rods to the positions saved in Main UI Position Slot 0.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!canScan);
  if (ImGui::Button("Start Scan", ImVec2(actionWidth, 0))) {
    if (training_.stepMm <= 0 || rods.empty()) trainingNotice_ = "Step must be positive and the channel must contain a Rod.";
    else if (std::abs(training_.startMm - training_.endMm) < 1e-9)
      trainingNotice_ = "Start and end must be different to produce a curve.";
    else {
      const auto selected = std::find_if(rods.begin(), rods.end(), [&](const ff::RodConfig &r) { return r.id == training_.trainingRod; });
      if (selected == rods.end() || std::abs(selected->p1) < 1e-12) trainingNotice_ = "Training Rod has invalid P0/P1 calibration.";
      else {
        const double lo = std::min(selected->p0 + selected->p1 * selected->softMin, selected->p0 + selected->p1 * selected->softMax);
        const double hi = std::max(selected->p0 + selected->p1 * selected->softMin, selected->p0 + selected->p1 * selected->softMax);
        if (training_.startMm < lo || training_.startMm > hi || training_.endMm < lo || training_.endMm > hi)
          trainingNotice_ = "Start and end must be within the selected Rod soft range.";
        else {
          training_.samples.clear(); training_.candidatesMm.clear(); training_.aims.clear(); trainingExecutionTargets_.clear();
          trainingForbiddenDirty_ = true;
          training_.complete = false; training_.machineId = config->machineId; training_.configVersion = config->configVersion;
          training_.createdUnixSeconds = std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::system_clock::now().time_since_epoch()).count();
          trainingNextMm_ = training_.startMm; trainingScanning_ = true; trainingMovePending_ = false; trainingStableSince_.reset(); trainingNotice_ = "Scanning...";
        }
      }
    }
  }
  tooltip("Move the selected Training Rod through the configured range and capture one energy sample at every step.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!trainingScanning_);
  if (ImGui::Button("Stop Scan", ImVec2(actionWidth, 0))) {
    service_.cancelMotion(); trainingScanning_ = false; trainingMovePending_ = false; trainingStableSince_.reset(); training_.complete = false;
    trainingNotice_ = "Scan stopped; partial samples were retained.";
  }
  tooltip("Stop the active scan and retain all samples already collected.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Save Session", ImVec2(actionWidth, 0))) {
    IGFD::FileDialogConfig dialogConfig;
    dialogConfig.path = trainingDirectory_;
    dialogConfig.fileName = automaticTrainingFileName(training_);
    ImGuiFileDialog::Instance()->OpenDialog(
        "TrainingSave", "Save Training Session", ".json",
        dialogConfig);
  }
  tooltip("Save scan samples, candidate lines, Rod positions, and training results using an automatic channel/source/time file name.");
  ImGui::SameLine();
  if (ImGui::Button("Load Session", ImVec2(actionWidth, 0))) ImGuiFileDialog::Instance()->OpenDialog("TrainingLoad", "Load Training Session", ".json", {trainingDirectory_});
  tooltip("Load a previously saved Training session for viewing, editing, and retraining.");

  if (training_.valleySettings.maximumCount <= 0) { training_.valleySettings.maximumCount = std::max(1, layer2TrainingRodCount); trainingSettingsChanged = true; }
  ImGui::BeginDisabled(training_.samples.size() < 3);
  if (ImGui::Button("Find Valleys", ImVec2(actionWidth, 0))) {
    const auto detected = ff::TrainingAnalysis::findValleys(training_.samples, training_.valleySettings);
    training_.candidatesMm.clear();
    for (double position : detected) {
      const double left = std::min(training_.startMm, training_.endMm) + 1.5;
      const double right = std::max(training_.startMm, training_.endMm) - 1.5;
      const bool separated = std::none_of(
          training_.candidatesMm.begin(), training_.candidatesMm.end(),
          [&](double existing) { return std::abs(existing - position) < 3.0; });
      if (position >= left && position <= right && separated)
        training_.candidatesMm.push_back(position);
    }
    training_.aims.clear(); trainingExecutionTargets_.clear(); trainingNotice_ = "Valley candidates generated. Edit them on the plot, then click Train.";
    trainingForbiddenDirty_ = true;
  }
  tooltip("Detect energy valleys and create editable candidate-position lines.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(training_.candidatesMm.empty() || !configuredTrainingRod);
  if (ImGui::Button("Clear Lines", ImVec2(actionWidth, 0))) {
    training_.candidatesMm.clear();
    training_.aims.clear();
    trainingExecutionTargets_.clear();
    trainingDraggingCandidate_ = -1;
    trainingForbiddenDirty_ = true;
    trainingNotice_ = "Candidate lines cleared.";
  }
  tooltip("Remove all candidate lines and discard the current training assignment.");
  ImGui::EndDisabled();

  std::vector<double> x, y;
  x.reserve(training_.samples.size()); y.reserve(training_.samples.size());
  for (const auto &sample : training_.samples) { x.push_back(sample.positionMm); y.push_back(static_cast<double>(sample.energy)); }
  // Right-click is reserved for deleting an obstruction line.  Disable the
  // built-in context menu (and box selection) so ImPlot never competes with
  // the Training editor's mouse gestures.
  if (ImPlot::BeginPlot("Energy vs Position", ImVec2(-1, 350),
                        ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect |
                            ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("Optical position (mm)", "Energy");
    if (!x.empty()) {
      const auto minmax = std::minmax_element(y.begin(), y.end());
      const double yMin = *minmax.first, yMax = std::max(yMin + 1.0, *minmax.second);
      // Same fixed-range behavior as the AFS S-curve: every acquisition or
      // edit redraws the full requested X interval and the data's padded Y
      // interval, so new data never remains outside the visible plot.
      const double yPad = std::max(1.0, (yMax - yMin) * .05);
      ImPlot::SetupAxesLimits(std::min(training_.startMm, training_.endMm),
                              std::max(training_.startMm, training_.endMm),
                              yMin - yPad, yMax + yPad, ImPlotCond_Always);
      ImPlot::PlotLine("Energy", x.data(), y.data(), static_cast<int>(x.size()));
      // Green candidate lines remain the sole aim marker.  Train adds only a
      // compact Rod label so the curve is not obscured by duplicated lines.
      int labelRow = 0;
      for (const auto &aim : training_.aims) {
        if (!aim.assigned) continue;
        const auto assignedRod = std::find_if(rods.begin(), rods.end(),
            [&](const ff::RodConfig &entry) { return entry.id == aim.address.rod; });
        if (assignedRod == rods.end()) continue;
        std::ostringstream label;
        label << "R" << int(aim.address.rod);
        const double labelY = yMax - (yMax - yMin) * .055 * (labelRow++ % 5);
        ImPlot::PlotText(label.str().c_str(), aim.requestedMm, labelY,
                         ImVec2(6, 0));
      }
      auto *draw = ImPlot::GetPlotDrawList();
      // Hit-test the visible green line itself.  A transparent DragLineX is
      // not reliable in the ImPlot version used by FF: once fully transparent
      // it may stop receiving mouse input, leaving a line that only looks
      // interactive.  Pixel-distance hit testing also keeps the grab width
      // constant at every X zoom level.
      int closest = -1;
      if (ImPlot::IsPlotHovered()) {
        const double mouse = ImPlot::GetPlotMousePos().x;
        constexpr float hitPixels = 8.0f;
        float distancePixels = hitPixels;
        const float mousePixels = ImGui::GetIO().MousePos.x;
        for (size_t i = 0; i < training_.candidatesMm.size(); ++i) {
          const float candidatePixels =
              ImPlot::PlotToPixels(training_.candidatesMm[i], yMin).x;
          const float candidateDistance =
              std::abs(candidatePixels - mousePixels);
          if (candidateDistance < distancePixels) {
            closest = static_cast<int>(i);
            distancePixels = candidateDistance;
          }
        }
        if (closest >= 0)
          ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && closest >= 0) {
          training_.candidatesMm.erase(training_.candidatesMm.begin() + closest);
          training_.aims.clear();
          trainingExecutionTargets_.clear();
          trainingDraggingCandidate_ = -1;
          trainingForbiddenDirty_ = true;
        }
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
          if (closest >= 0) {
            trainingDraggingCandidate_ = closest;
            trainingDragOriginalMm_ = training_.candidatesMm[closest];
            trainingDragLastSafeMm_ = training_.candidatesMm[closest];
          } else if (trainingCandidateCapacityReached) {
            trainingNotice_ = "Maximum candidate count reached (" +
                std::to_string(layer2TrainingRodCount) +
                "). Delete a line before adding another.";
          } else {
            auto trial = training_.candidatesMm; trial.push_back(mouse); std::string reason;
            if (candidateSetIsLegal(trial, &reason)) {
              training_.candidatesMm = std::move(trial);
              training_.aims.clear(); trainingExecutionTargets_.clear(); trainingForbiddenDirty_ = true;
            }
            else trainingNotice_ = "Candidate rejected: " + reason;
          }
        }
      }
      if (trainingDraggingCandidate_ >= 0 &&
          ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        const auto movingIndex =
            static_cast<size_t>(trainingDraggingCandidate_);
        if (movingIndex < training_.candidatesMm.size()) {
          const double proposed = ImPlot::GetPlotMousePos().x;
          const auto interval = ff::TrainingAnalysis::dragInterval(
              training_.candidatesMm, movingIndex, trainingDragOriginalMm_,
              training_.startMm, training_.endMm, 3.0);
          if (interval.valid && proposed >= interval.minimumMm &&
              proposed <= interval.maximumMm) {
            training_.candidatesMm[movingIndex] = proposed;
            trainingDragLastSafeMm_ = proposed;
          } else {
            training_.candidatesMm[movingIndex] = trainingDragLastSafeMm_;
          }
          training_.aims.clear();
          trainingExecutionTargets_.clear();
          trainingForbiddenDirty_ = true;
        }
      }
      if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && trainingDraggingCandidate_ >= 0) {
        std::string reason;
        if (candidateSetIsLegal(training_.candidatesMm, &reason)) {
          trainingForbiddenDirty_ = true;
        } else {
          training_.candidatesMm[trainingDraggingCandidate_] = trainingDragOriginalMm_;
          trainingForbiddenDirty_ = true;
          trainingNotice_ = "Candidate move rejected: " + reason;
        }
        trainingDraggingCandidate_ = -1;
      }
      refreshTrainingForbiddenBands();
      for (const auto &[begin, end] : trainingForbiddenBands_)
        draw->AddRectFilled(ImPlot::PlotToPixels(begin, yMin),
                            ImPlot::PlotToPixels(end, yMax),
                            IM_COL32(210, 45, 45, 70));
      const ImU32 candidateColor = trainingCandidateCapacityReached
          ? IM_COL32(255, 214, 40, 255)
          : IM_COL32(51, 242, 89, 255);
      for (double candidate : training_.candidatesMm)
        draw->AddLine(ImPlot::PlotToPixels(candidate, yMin),
                      ImPlot::PlotToPixels(candidate, yMax),
                      candidateColor, 2.0f);
    }
    ImPlot::EndPlot();
  }
  ImGui::TextDisabled("Left-click: add/select; drag: move; right-click: delete.");
  if (trainingCandidateCapacityReached)
    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.16f, 1.0f),
                       "Maximum candidate count reached (%d). Delete a line before adding another.",
                       layer2TrainingRodCount);
  ImGui::BeginDisabled(training_.candidatesMm.empty());
  if (ImGui::Button("Train", ImVec2(actionWidth, 0))) updateTrainingAssignment();
  tooltip("Assign Layer 2 non-IRIS Rods to candidate positions, excluding the Training Rod, and plan its evasion move.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(trainingExecutionTargets_.empty());
  if (ImGui::Button("Execute", ImVec2(actionWidth, 0))) {
    // Use the same complete-target submission path as Main UI "load".  The
    // service refreshes and validates the target state again immediately
    // before any command is sent.
    auto result = service_.setRodTargets(trainingExecutionTargets_);
    if (result) {
      service_.refresh();
      for (auto &mainTarget : targets_) {
        const auto trainedTarget = std::find_if(
            trainingExecutionTargets_.begin(), trainingExecutionTargets_.end(),
            [&](const ff::RodTarget &entry) {
              return entry.address == mainTarget.address;
            });
        if (trainedTarget != trainingExecutionTargets_.end())
          mainTarget = *trainedTarget;
      }
      trainingNotice_ =
          "Training targets were sent and Main UI targets were updated.";
    } else trainingNotice_ = "Execute failed: " + result.message;
  }
  tooltip("Refresh and validate the complete channel target state, then send all Rod targets synchronously like Main UI Load.");
  ImGui::EndDisabled();
  ImGui::SeparatorText("Layer 2 Rod Positions and Training Aims");
  if (ImGui::BeginTable("TrainingAims", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Rod"); ImGui::TableSetupColumn("Current mm"); ImGui::TableSetupColumn("Train Aim mm"); ImGui::TableSetupColumn("Safe / Evasion mm"); ImGui::TableSetupColumn("Assignment"); ImGui::TableHeadersRow();
    for (size_t index = 0; index < rods.size(); ++index) {
      const auto &rodConfig = rods[index];
      bool inLayer2 = false, irisRelated = false;
      for (const auto &member : config->channels[channel].layerMembers) {
        if (member.rod != rodConfig.id) continue;
        inLayer2 = inLayer2 || member.layer == 2;
        irisRelated = irisRelated || member.layer != 2;
      }
      if (!inLayer2 || irisRelated) continue;
      const ff::RodStatus *feedback = index < currentStatus.channels[channel].rods.size() ? &currentStatus.channels[channel].rods[index] : nullptr;
      const auto aim = std::find_if(training_.aims.begin(), training_.aims.end(), [&](const ff::TrainingAim &entry) { return entry.address.rod == rodConfig.id; });
      const bool isEvasionRod = trainingEvasionTarget_ && !training_.aims.empty() && trainingEvasionTarget_->address.rod == rodConfig.id && trainingEvasionTarget_->address.channel == training_.channel;
      ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("Rod %d", int(rodConfig.id));
      ImGui::TableSetColumnIndex(1); if (feedback) ImGui::Text("%.4f", rodConfig.p0 + rodConfig.p1 * feedback->actualPosition); else ImGui::TextUnformatted("--");
      ImGui::TableSetColumnIndex(2); if (aim != training_.aims.end() && aim->assigned) ImGui::Text("%.4f", aim->quantizedMm); else ImGui::TextUnformatted("--");
      ImGui::TableSetColumnIndex(3);
      if (isEvasionRod) ImGui::Text("%.4f", rodConfig.p0 + rodConfig.p1 * trainingEvasionTarget_->position);
      else ImGui::TextUnformatted("--");
      ImGui::TableSetColumnIndex(4);
      if (isEvasionRod) ImGui::TextUnformatted(
          trainingExecutionTargets_.empty() ? "training rod / evasion aim"
                                            : "training rod / ready");
      else ImGui::TextUnformatted(
          aim != training_.aims.end() && aim->assigned
              ? (trainingExecutionTargets_.empty() ? "assigned / aim only"
                                                   : "assigned / ready")
              : "unassigned / stay");
    }
    ImGui::EndTable();
  }
  const bool capacityNotice = trainingNotice_.rfind(
      "Maximum candidate count reached", 0) == 0;
  if (!trainingNotice_.empty() && !capacityNotice)
    ImGui::TextWrapped("%s", trainingNotice_.c_str());
  if (ImGuiFileDialog::Instance()->Display("TrainingSave")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      training_.executionTargets = trainingExecutionTargets_;
      std::ostringstream summary;
      summary << "CH" << (static_cast<int>(training_.channel) + 1)
              << "; Training Rod R" << int(training_.trainingRod)
              << "; " << training_.candidatesMm.size()
              << " obstruction position(s)";
      if (!training_.candidatesMm.empty()) {
        summary << " at ";
        for (size_t index = 0; index < training_.candidatesMm.size(); ++index) {
          if (index) summary << ", ";
          summary << std::fixed << std::setprecision(2)
                  << training_.candidatesMm[index] << " mm";
        }
      }
      std::vector<int> assignedRods;
      for (const auto &aim : training_.aims)
        if (aim.assigned) assignedRods.push_back(int(aim.address.rod));
      if (!assignedRods.empty()) {
        summary << "; assigned ";
        for (size_t index = 0; index < assignedRods.size(); ++index) {
          if (index) summary << ", ";
          summary << "R" << assignedRods[index];
        }
      }
      summary << (training_.executionTargets.empty()
                      ? "; no executable target set"
                      : "; executable target set included");
      training_.occlusionSummary = summary.str();
      const auto current = service_.status();
      const auto &savedChannel = config->channels[static_cast<int>(training_.channel)];
      training_.rodStates.clear();
      for (size_t i = 0; i < savedChannel.rods.size() &&
                         i < current.channels[static_cast<int>(training_.channel)].rods.size();
           ++i) {
        const auto &feedback =
            current.channels[static_cast<int>(training_.channel)].rods[i];
        training_.rodStates.push_back(
            {{training_.channel, savedChannel.rods[i].id},
             feedback.actualPosition, feedback.targetPosition});
      }
      for (size_t i = 0; i < savedChannel.rods.size() && i < current.channels[static_cast<int>(training_.channel)].rods.size(); ++i) {
        const auto &rodConfig = savedChannel.rods[i];
        const auto actual = current.channels[static_cast<int>(training_.channel)].rods[i].actualPosition;
        auto aim = std::find_if(training_.aims.begin(), training_.aims.end(),
                                [&](const ff::TrainingAim &entry) { return entry.address.rod == rodConfig.id; });
        if (aim == training_.aims.end())
          training_.aims.push_back({{training_.channel, rodConfig.id},
                                    rodConfig.p0 + rodConfig.p1 * actual,
                                    rodConfig.p0 + rodConfig.p1 * actual,
                                    actual, actual, false});
        else {
          aim->actualEncoder = actual;
          if (!aim->assigned) {
            aim->encoderAim = actual;
            aim->requestedMm = aim->quantizedMm = rodConfig.p0 + rodConfig.p1 * actual;
          }
        }
      }
      auto saved = ff::TrainingAnalysis::saveSession(
          ImGuiFileDialog::Instance()->GetFilePathName(), training_);
      trainingNotice_ = saved ? "Training session saved." : saved.message;
    }
    ImGuiFileDialog::Instance()->Close();
  }
  if (ImGuiFileDialog::Instance()->Display("TrainingLoad")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      auto loaded = ff::TrainingAnalysis::loadSession(ImGuiFileDialog::Instance()->GetFilePathName());
      if (loaded) {
        training_ = std::move(loaded.value);
        training_.source = ff::TrainingSource::Loaded;
        trainingExecutionTargets_.clear();
        trainingEvasionTarget_.reset();
        trainingForbiddenDirty_ = true;
        const int loadedChannel = std::clamp(static_cast<int>(training_.channel), 0, 2);
        training_.channel = static_cast<ff::ChannelId>(loadedChannel);
        trainingSessions_[loadedChannel] = training_;
        if (training_.machineId != config->machineId || training_.configVersion != config->configVersion) { training_.aims.clear(); trainingNotice_ = "Session loaded for viewing; machine/config differs, so Train is required before Execute."; }
        else trainingNotice_ = "Training session loaded; click Train to validate current hardware state.";
      } else trainingNotice_ = loaded.message;
    }
    ImGuiFileDialog::Instance()->Close();
  }
  if (trainingSettingsChanged)
    saveUiSettings();
  ImGui::End();
}

void MainUi::drawChannel(int index, const ff::FFStatus &s) {
  auto &c = s.channels[index];
  ImGui::Text("CH%d: %zu configured device(s)", index + 1, c.rods.size());
}

void MainUi::initializeParameters() {
  if (parametersInitialized_)
    return;
  auto loaded = ff::FFConfigIO::load(configPath_);
  if (!loaded) {
    parameterNotice_ = loaded.message;
    return;
  }
  parameterOriginal_ = parameterDraft_ = std::move(loaded.value);
  parametersDirty_ = false;
  parametersInitialized_ = true;
}

void MainUi::markParametersDirty() { parametersDirty_ = true; }

bool MainUi::saveParameterFile(const std::string &path) {
  auto saved = ff::FFConfigIO::save(path, parameterDraft_);
  if (!saved) { parameterNotice_ = saved.message; return false; }
  configPath_ = path;
  parameterOriginal_ = parameterDraft_;
  parametersDirty_ = false;
  saveUiSettings();
  parameterNotice_ = "Configuration saved.";
  return true;
}

bool MainUi::loadParameterFile(const std::string &path) {
  auto loaded = ff::FFConfigIO::load(path);
  if (!loaded) { parameterNotice_ = loaded.message; return false; }
  parameterDraft_ = std::move(loaded.value);
  parameterOriginal_ = parameterDraft_;
  configPath_ = path;
  parametersDirty_ = false;
  selectedRod_ = 0;
  parameterNotice_ = "Configuration loaded into the draft.";
  return true;
}

void MainUi::parameters() {
  initializeParameters();
  if (!parametersInitialized_) return;
  bool open = showParameters_;
  ImGui::SetNextWindowSize(ImVec2(980, 700), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("FF_Config", &open)) { ImGui::End(); return; }
  if (!open && parametersDirty_) {
    ImGui::OpenPopup("Discard unsaved configuration?");
    open = true;
  }
  showParameters_ = open;
  ImGui::TextDisabled("Current file: %s", configPath_.c_str());
  if (ImGui::Button("Load")) {
    if (parametersDirty_) { pendingLoadPath_ = "__open_dialog__"; ImGui::OpenPopup("Discard unsaved configuration?"); }
    else ImGuiFileDialog::Instance()->OpenDialog("FFConfigLoad", "Load FF Configuration", ".json", {});
  }
  tooltip("Load a JSON configuration into the editable draft. Unsaved changes are protected.");
  ImGui::SameLine();
  if (ImGui::Button("Save")) saveParameterFile(configPath_);
  tooltip("Validate and save the current draft to the displayed configuration file. This does not apply it to FF.");
  ImGui::SameLine();
  if (ImGui::Button("Save As")) ImGuiFileDialog::Instance()->OpenDialog("FFConfigSave", "Save FF Configuration", ".json", {});
  tooltip("Validate and save the current draft to a new JSON file.");
  ImGui::SameLine();
  if (ImGui::Button("Validate")) {
    auto checked = ff::FFConfigIO::validate(parameterDraft_);
    parameterNotice_ = checked ? "Configuration is valid." : checked.message;
  }
  tooltip("Check all limits, Rod IDs, layer topology, and clearance rules without saving or applying changes.");
  ImGui::SameLine();
  ImGui::BeginDisabled(service_.connected());
  if (ImGui::Button("Apply")) {
    auto applied = service_.applyConfig(parameterDraft_, simulation_, serialPorts_, enabledChannelCount_);
    if (applied) {
      parameterOriginal_ = parameterDraft_;
      configLoaded_ = true; targetsInitialized_ = false; saveUiSettings();
      parameterNotice_ = "Configuration applied. Open FF to connect.";
    } else parameterNotice_ = applied.message;
  }
  tooltip("Replace the disconnected FF runtime configuration with this validated draft. Save separately to persist it on disk.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Revert")) { parameterDraft_ = parameterOriginal_; parametersDirty_ = false; }
  tooltip("Discard draft changes and restore the configuration as it was last loaded or saved.");
  if (service_.connected()) ImGui::TextColored(ImVec4(1, .7f, .2f, 1), "Disconnect FF before Apply.");
  if (!parameterNotice_.empty()) ImGui::TextWrapped("%s", parameterNotice_.c_str());
  ImGui::Separator();

  if (ImGui::CollapsingHeader("System", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (inputString("Machine ID", parameterDraft_.machineId) || inputString("Config Version", parameterDraft_.configVersion)) markParametersDirty();
    int maxRods = static_cast<int>(parameterDraft_.maxActiveRodCount);
    int expiration = static_cast<int>(parameterDraft_.statusExpiration.count());
    if (ImGui::InputInt("Max Active Rod Count", &maxRods) || ImGui::InputInt("Status Expiration (ms)", &expiration)) {
      parameterDraft_.maxActiveRodCount = std::max(0, maxRods);
      parameterDraft_.statusExpiration = std::chrono::milliseconds(std::max(0, expiration)); markParametersDirty();
    }
  }
  if (!ImGui::BeginTabBar("ParameterChannels")) { ImGui::End(); return; }
  for (int channelIndex = 0; channelIndex < 3; ++channelIndex) {
    const std::string label = "CH" + std::to_string(channelIndex + 1);
    if (!ImGui::BeginTabItem(label.c_str())) continue;
    parameterChannel_ = channelIndex;
    auto &channel = parameterDraft_.channels[channelIndex];
    if (ImGui::CollapsingHeader("Controller", ImGuiTreeNodeFlags_DefaultOpen)) {
      auto &controller = channel.controller;
      if (inputString("Controller Name", controller.name) || inputString("Config Serial Port", controller.serial.portName)) markParametersDirty();
      int baud = static_cast<int>(controller.serial.baudRate), timeout = static_cast<int>(controller.serial.timeout.count());
      int poll = static_cast<int>(controller.pollPeriod.count()), retry = static_cast<int>(controller.retryCount);
      if (ImGui::InputInt("Baud Rate", &baud) || ImGui::InputInt("Timeout (ms)", &timeout) || ImGui::InputInt("Poll Period (ms)", &poll) || ImGui::InputInt("Retry Count", &retry)) {
        controller.serial.baudRate = std::max(0, baud); controller.serial.timeout = std::chrono::milliseconds(std::max(0, timeout));
        controller.pollPeriod = std::chrono::milliseconds(std::max(0, poll)); controller.retryCount = std::max(0, retry); markParametersDirty();
      }
      ImGui::TextDisabled("Connection settings on the Welcome page override this port at runtime.");
    }
    if (ImGui::CollapsingHeader("Training", ImGuiTreeNodeFlags_DefaultOpen)) {
      std::set<int> layer2Rods;
      for (const auto &member : channel.layerMembers) {
        if (member.layer == 2 && std::any_of(channel.rods.begin(), channel.rods.end(),
            [&member](const ff::RodConfig &rod) { return rod.id == member.rod; })) {
          layer2Rods.insert(static_cast<int>(member.rod));
        }
      }
      const int selectedTrainingRod = channel.trainingRod
          ? static_cast<int>(*channel.trainingRod) : -1;
      const std::string preview = selectedTrainingRod < 0
          ? "Not configured" : "Rod " + std::to_string(selectedTrainingRod);
      if (ImGui::BeginCombo("Training Rod", preview.c_str())) {
        if (ImGui::Selectable("Not configured", selectedTrainingRod < 0)) {
          channel.trainingRod.reset();
          markParametersDirty();
        }
        for (const int rodId : layer2Rods) {
          const std::string option = "Rod " + std::to_string(rodId);
          if (ImGui::Selectable(option.c_str(), selectedTrainingRod == rodId)) {
            channel.trainingRod = static_cast<ff::RodId>(rodId);
            markParametersDirty();
          }
        }
        ImGui::EndCombo();
      }
      tooltip("Select the Layer 2 Rod used for Training scans. This is saved as training_rod_id in FF configuration.");
      if (layer2Rods.empty())
        ImGui::TextDisabled("Add a valid Layer 2 membership before selecting a Training Rod.");
    }
    if (!ImGui::BeginTabBar("ConfigSections")) { ImGui::EndTabItem(); continue; }
    if (ImGui::BeginTabItem("Rods")) {
    if (ImGui::BeginTable("Rods", 8, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp, ImVec2(0, 190))) {
      ImGui::TableSetupScrollFreeze(2, 1);
      ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 45); ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupColumn("Soft Min"); ImGui::TableSetupColumn("Soft Max"); ImGui::TableSetupColumn("Safe"); ImGui::TableSetupColumn("P0"); ImGui::TableSetupColumn("P1"); ImGui::TableSetupColumn("Max Vel."); ImGui::TableHeadersRow();
      for (size_t i = 0; i < channel.rods.size(); ++i) {
        const auto &rod = channel.rods[i]; ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0);
        if (ImGui::Selectable(std::to_string(int(rod.id)).c_str(), selectedRod_ == int(i), ImGuiSelectableFlags_SpanAllColumns)) selectedRod_ = int(i);
        ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(rod.name.c_str()); ImGui::TableSetColumnIndex(2); ImGui::Text("%d", rod.softMin); ImGui::TableSetColumnIndex(3); ImGui::Text("%d", rod.softMax);
        ImGui::TableSetColumnIndex(4); ImGui::Text("%d", rod.safePosition); ImGui::TableSetColumnIndex(5); ImGui::Text("%.5f", rod.p0); ImGui::TableSetColumnIndex(6); ImGui::Text("%.7f", rod.p1); ImGui::TableSetColumnIndex(7); ImGui::Text("%.2f", rod.maxVelocity);
      }
      ImGui::EndTable();
    }
    if (ImGui::Button("Add Rod")) {
      int id = 0; while (id < 256 && std::any_of(channel.rods.begin(), channel.rods.end(), [id](const ff::RodConfig &r){ return int(r.id) == id; })) ++id;
      if (id < 256) { ff::RodConfig rod; rod.id = static_cast<ff::RodId>(id); rod.name = label + "-R" + std::to_string(id); rod.hardMax = rod.softMax = 4095; rod.p1 = 1; rod.maxVelocity = 1; channel.rods.push_back(rod); selectedRod_ = int(channel.rods.size()) - 1; markParametersDirty(); }
    }
    tooltip("Add a Rod with the next available ID to this channel.");
    ImGui::SameLine();
    if (ImGui::Button("Remove Selected Rod") && selectedRod_ >= 0 && static_cast<size_t>(selectedRod_) < channel.rods.size()) {
      const int id = int(channel.rods[selectedRod_].id);
      const bool referenced = std::any_of(channel.layerMembers.begin(), channel.layerMembers.end(), [id](const ff::LayerMembership &m){ return int(m.rod) == id || m.leftNeighbor == id || m.rightNeighbor == id; }) || std::any_of(channel.clearanceRules.begin(), channel.clearanceRules.end(), [id](const ff::PairClearanceRule &r){ return int(r.first) == id || int(r.second) == id; });
      if (referenced) parameterNotice_ = "Remove layer memberships and clearance rules that reference this Rod first.";
      else { eraseAt(channel.rods, selectedRod_); selectedRod_ = std::max(0, selectedRod_ - 1); markParametersDirty(); }
    }
    tooltip("Remove the selected Rod. A Rod referenced by a layer or clearance rule must be unlinked first.");
    if (selectedRod_ >= 0 && static_cast<size_t>(selectedRod_) < channel.rods.size()) {
      auto &rod = channel.rods[selectedRod_]; ImGui::SeparatorText("Selected Rod"); ImGui::Text("Rod ID: %d", int(rod.id));
      bool changed = inputString("Rod Name", rod.name); changed |= ImGui::InputInt("Hard Min", &rod.hardMin) || ImGui::InputInt("Hard Max", &rod.hardMax) || ImGui::InputInt("Soft Min", &rod.softMin) || ImGui::InputInt("Soft Max", &rod.softMax) || ImGui::InputInt("Zero Position", &rod.zeroPosition) || ImGui::InputInt("Safe Position", &rod.safePosition) || ImGui::Checkbox("Safe Position Valid", &rod.safePositionValid) || ImGui::InputDouble("P0", &rod.p0) || ImGui::InputDouble("P1", &rod.p1) || ImGui::InputInt("Inward Direction", &rod.inwardDirection) || ImGui::InputDouble("Max Velocity", &rod.maxVelocity) || ImGui::InputDouble("Max Acceleration", &rod.maxAcceleration) || ImGui::InputDouble("Max Deceleration", &rod.maxDeceleration) || ImGui::InputInt("Position Tolerance", &rod.positionTolerance);
      int stable = int(rod.stableTime.count()), moveTimeout = int(rod.moveTimeout.count()); changed |= ImGui::InputInt("Stable Time (ms)", &stable) || ImGui::InputInt("Move Timeout (ms)", &moveTimeout); rod.stableTime = std::chrono::milliseconds(std::max(0, stable)); rod.moveTimeout = std::chrono::milliseconds(std::max(0, moveTimeout)); if (changed) markParametersDirty();
    }
    ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Layer Memberships")) {
    ImGui::SeparatorText("Layer Memberships");
    if (ImGui::BeginTable("LayerMembershipTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp, ImVec2(0, 130))) {
      ImGui::TableSetupColumn("Rod"); ImGui::TableSetupColumn("Layer"); ImGui::TableSetupColumn("Role"); ImGui::TableSetupColumn("Left Neighbor"); ImGui::TableSetupColumn("Right Neighbor"); ImGui::TableHeadersRow();
      for (size_t i = 0; i < channel.layerMembers.size(); ++i) {
        const auto &m = channel.layerMembers[i]; ImGui::PushID(static_cast<int>(i)); ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0);
        if (ImGui::Selectable(std::to_string(int(m.rod)).c_str(), selectedLayerMember_ == int(i), ImGuiSelectableFlags_SpanAllColumns)) selectedLayerMember_ = int(i);
        ImGui::TableSetColumnIndex(1); ImGui::Text("%d", m.layer); ImGui::TableSetColumnIndex(2); ImGui::Text("%d", m.role); ImGui::TableSetColumnIndex(3); ImGui::Text("%d", m.leftNeighbor); ImGui::TableSetColumnIndex(4); ImGui::Text("%d", m.rightNeighbor);
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    if (ImGui::Button("Add Layer Member")) { channel.layerMembers.push_back({}); selectedLayerMember_ = int(channel.layerMembers.size()) - 1; markParametersDirty(); }
    tooltip("Add a layer-membership record for this channel.");
    ImGui::SameLine();
    if (ImGui::Button("Remove Selected Member") && eraseAt(channel.layerMembers, selectedLayerMember_)) { selectedLayerMember_ = std::max(0, selectedLayerMember_ - 1); markParametersDirty(); }
    tooltip("Remove the selected layer-membership record.");
    if (selectedLayerMember_ >= 0 && static_cast<size_t>(selectedLayerMember_) < channel.layerMembers.size()) {
      auto &m = channel.layerMembers[selectedLayerMember_]; ImGui::Text("Selected Layer Membership"); int rod = int(m.rod);
      bool changed = ImGui::InputInt("Member Rod", &rod) || ImGui::InputInt("Member Layer", &m.layer) || ImGui::InputInt("Member Role", &m.role) || ImGui::InputInt("Member Left Neighbor", &m.leftNeighbor) || ImGui::InputInt("Member Right Neighbor", &m.rightNeighbor);
      if (changed) { m.rod = static_cast<ff::RodId>(std::clamp(rod, 0, 255)); markParametersDirty(); }
    }
    ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Clearance Rules")) {
    ImGui::SeparatorText("Clearance Rules");
    if (ImGui::BeginTable("ClearanceRuleTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp, ImVec2(0, 120))) {
      ImGui::TableSetupColumn("Layer"); ImGui::TableSetupColumn("Rod A"); ImGui::TableSetupColumn("Rod B"); ImGui::TableSetupColumn("Minimum Clearance (mm)"); ImGui::TableHeadersRow();
      for (size_t i = 0; i < channel.clearanceRules.size(); ++i) {
        const auto &r = channel.clearanceRules[i]; ImGui::PushID(static_cast<int>(i)); ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0);
        if (ImGui::Selectable(std::to_string(r.layer).c_str(), selectedClearanceRule_ == int(i), ImGuiSelectableFlags_SpanAllColumns)) selectedClearanceRule_ = int(i);
        ImGui::TableSetColumnIndex(1); ImGui::Text("%d", int(r.first)); ImGui::TableSetColumnIndex(2); ImGui::Text("%d", int(r.second)); ImGui::TableSetColumnIndex(3); ImGui::Text("%.4f", r.minimumClearanceMm);
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    if (ImGui::Button("Add Clearance Rule")) { channel.clearanceRules.push_back({}); selectedClearanceRule_ = int(channel.clearanceRules.size()) - 1; markParametersDirty(); }
    tooltip("Add a minimum-clearance rule for a pair of Rods.");
    ImGui::SameLine();
    if (ImGui::Button("Remove Selected Rule") && eraseAt(channel.clearanceRules, selectedClearanceRule_)) { selectedClearanceRule_ = std::max(0, selectedClearanceRule_ - 1); markParametersDirty(); }
    tooltip("Remove the selected clearance rule.");
    if (selectedClearanceRule_ >= 0 && static_cast<size_t>(selectedClearanceRule_) < channel.clearanceRules.size()) {
      auto &r = channel.clearanceRules[selectedClearanceRule_]; ImGui::Text("Selected Clearance Rule"); int first = int(r.first), second = int(r.second);
      bool changed = ImGui::InputInt("Rule Layer", &r.layer) || ImGui::InputInt("Rule Rod A", &first) || ImGui::InputInt("Rule Rod B", &second) || ImGui::InputDouble("Rule Minimum Clearance (mm)", &r.minimumClearanceMm);
      if (changed) { r.first = static_cast<ff::RodId>(std::clamp(first, 0, 255)); r.second = static_cast<ff::RodId>(std::clamp(second, 0, 255)); markParametersDirty(); }
    }
    ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::EndTabItem();
  }
  ImGui::EndTabBar();
  if (ImGuiFileDialog::Instance()->Display("FFConfigLoad")) { if (ImGuiFileDialog::Instance()->IsOk()) loadParameterFile(ImGuiFileDialog::Instance()->GetFilePathName()); ImGuiFileDialog::Instance()->Close(); }
  if (ImGuiFileDialog::Instance()->Display("FFConfigSave")) { if (ImGuiFileDialog::Instance()->IsOk()) saveParameterFile(ImGuiFileDialog::Instance()->GetFilePathName()); ImGuiFileDialog::Instance()->Close(); }
  if (ImGui::BeginPopupModal("Discard unsaved configuration?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) { ImGui::Text("Save changes before continuing?"); if (ImGui::Button("Save")) { if (saveParameterFile(configPath_) && pendingLoadPath_ == "__open_dialog__") ImGuiFileDialog::Instance()->OpenDialog("FFConfigLoad", "Load FF Configuration", ".json", {}); pendingLoadPath_.clear(); ImGui::CloseCurrentPopup(); } ImGui::SameLine(); if (ImGui::Button("Discard")) { parameterDraft_ = parameterOriginal_; parametersDirty_ = false; if (pendingLoadPath_ == "__open_dialog__") ImGuiFileDialog::Instance()->OpenDialog("FFConfigLoad", "Load FF Configuration", ".json", {}); else showParameters_ = false; pendingLoadPath_.clear(); ImGui::CloseCurrentPopup(); } ImGui::SameLine(); if (ImGui::Button("Cancel")) { pendingLoadPath_.clear(); ImGui::CloseCurrentPopup(); } ImGui::EndPopup(); }
  ImGui::End();
}
} // namespace ff_gui
