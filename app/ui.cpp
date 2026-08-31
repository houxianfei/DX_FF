#include "ff_gui.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
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
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 36.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
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
  refreshPresetSlots();
  loadUiSettings();
  scanSerialPorts();
  ensureConfig();
}

void MainUi::refreshPresetSlots() {
  presetSlots_.clear();
  const std::regex fileNamePattern(R"(^slot_([0-9]+)\.csv$)");
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
                                        ".csv"))
      .string();
}

bool MainUi::savePreset(int slot,
                        const std::vector<ff::RodTarget> &targets) {
  std::ofstream output(presetPath(slot), std::ios::trunc);
  if (!output)
    return false;
  output << "channel,rod,position,velocity\n";
  for (const auto &target : targets)
    output << static_cast<int>(target.address.channel) << ','
           << static_cast<int>(target.address.rod) << ',' << target.position
           << ',' << target.velocity << '\n';
  return static_cast<bool>(output);
}

std::vector<ff::RodTarget> MainUi::loadPreset(int slot) const {
  std::ifstream input(presetPath(slot));
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
    }
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

  int highestSlot = -1;
  if (!presetSlots_.empty())
    highestSlot = presetSlots_.back();
  const int newSlot = highestSlot + 1;
  const auto selectedSlotLabel = "Slot " + std::to_string(selectedPresetSlot_);
  ImGui::SetNextItemWidth(120.0f);
  if (ImGui::BeginCombo("Position Slot", selectedSlotLabel.c_str())) {
    for (int slot : presetSlots_) {
      const auto label = "Slot " + std::to_string(slot);
      if (ImGui::Selectable(label.c_str(), selectedPresetSlot_ == slot))
        selectedPresetSlot_ = slot;
    }
    const auto newSlotLabel = "Slot " + std::to_string(newSlot) + " (new)";
    if (ImGui::Selectable(newSlotLabel.c_str(), selectedPresetSlot_ == newSlot))
      selectedPresetSlot_ = newSlot;
    ImGui::EndCombo();
  }
  ImGui::SameLine();
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
  ImGui::SameLine();
  ImGui::BeginDisabled(!hasPresetSlot(selectedPresetSlot_));
  if (ImGui::Button("load")) {
    auto preset = loadPreset(selectedPresetSlot_);
    if (preset.empty()) {
      notice_ = "The selected position preset is invalid or empty.";
    } else {
      auto x = service_.setRodTargets(preset);
      if (x) {
        targets_ = std::move(preset);
        notice_ = "Position slot " + std::to_string(selectedPresetSlot_) +
                  " loaded.";
      } else {
        notice_ = x.message + "; restored to the current safe positions.";
      }
    }
  }
  ImGui::EndDisabled();

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
