#include "ff_gui.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
namespace ff_gui {
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
  loadUiSettings();
  scanSerialPorts();
  ensureConfig();
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
    if (!needsMigration) {
      auto existing = ff::FFConfigIO::load(configPath_);
      const size_t expected[3] = {14, 14, 12};
      if (existing) {
        for (int i = 0; i < 3; ++i)
          needsMigration =
              needsMigration ||
              existing.value.channels[i].rods.size() != expected[i];
      }
    }
  }
  if (needsMigration)
    ff::FFConfigIO::save(configPath_, defaultConfig());
  auto x = service_.loadConfig(configPath_, simulation_, serialPorts_,
                               enabledChannelCount_);
  configLoaded_ = static_cast<bool>(x);
  targetsInitialized_ = false;
  plan_ = {};
  if (!x)
    notice_ = x.message;
}
void MainUi::draw() {
  if (!configLoaded_ || !service_.connected()) {
    welcome();
    return;
  }
  dashboard();
}
void MainUi::welcome() {
  static const char *modes[] = {"Simulation", "Real Hardware"};
  static const char *channelCounts[] = {"1", "2", "3"};
  constexpr float controlWidth = 240.0f;
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
  if (!notice_.empty())
    ImGui::TextColored(ImVec4(1, 0.5f, 0.2f, 1), "%s", notice_.c_str());
}
void MainUi::dashboard() {
  auto s = service_.status();
  ImGui::Text("FF GUI 1.0.0   |   Mode: %s",
              simulation_ ? "Simulation" : "Real");

  if (ImGui::Button(service_.connected() ? "Disconnect" : "Connect")) {
    auto x = service_.connected() ? service_.disconnect() : service_.connect();
    if (!x)
      notice_ = x.message;
  }
  ImGui::SameLine();
  if (ImGui::Button("Refresh Status")) {
    auto x = service_.refresh();
    if (!x)
      notice_ = x.message;
  }
  ImGui::SameLine();
  if (ImGui::Button("EMERGENCY STOP"))
    service_.emergencyStop();
  ImGui::Separator();
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
  ImGui::SeparatorText("Targets and Planning");
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
  for (auto &r : targets_) {
    if (static_cast<int>(r.address.channel) != activeChannel_)
      continue;
    const auto rodIndex = static_cast<size_t>(r.address.rod);
    const auto &current = s.channels[activeChannel_].rods[rodIndex];
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

    const float targetColumnX = ImGui::GetCursorPosX() + 175.0f;
    ImGui::Text("Rod %2d | Actual: %7.3f%s", r.address.rod,
                toOpticalPosition(current.actualPosition),
                current.alarm ? "  ALARM" : "");
    ImGui::SameLine(targetColumnX);
    ImGui::SetNextItemWidth(360.0f);
    if (ImGui::SliderFloat(("Target##CH" +
                            std::to_string(int(r.address.channel) + 1) + "R" +
                            std::to_string(r.address.rod))
                               .c_str(),
                           &targetMm, minimumMm, maximumMm, "%.3f")) {
      r.position =
          std::clamp(toEncoder(targetMm), config->softMin, config->softMax);
    }
  }
  if (ImGui::Button("Preview Plan")) {
    auto x = service_.previewPlan({targets_, false});
    if (x)
      plan_ = x.value;
    else
      notice_ = x.message;
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(!plan_.valid);
  if (ImGui::Button("Execute Plan")) {
    auto x = service_.executePlan(plan_);
    if (!x)
      notice_ = x.message;
  }
  ImGui::EndDisabled();
  if (plan_.valid)
    ImGui::Text("Plan #%llu: %s, %zu steps", (unsigned long long)plan_.id,
                plan_.message.c_str(), plan_.steps.size());
  if (!notice_.empty())
    ImGui::TextColored(ImVec4(1, 0.5f, 0.2f, 1), "%s", notice_.c_str());
}
void MainUi::drawChannel(int index, const ff::FFStatus &s) {
  auto &c = s.channels[index];
  ImGui::Text("CH%d  %s", index + 1, c.connected ? "Connected" : "Offline");
  auto &safe = s.safety[index];
  ImVec4 col = safe.state == ff::SafetyState::Safe ? ImVec4(0.2f, 0.9f, 0.3f, 1)
                                                   : ImVec4(1, 0.3f, 0.2f, 1);
  ImGui::TextColored(col, "Safety: %s",
                     safe.state == ff::SafetyState::Safe ? "SAFE" : "UNSAFE");
  ImGui::TextDisabled("Real-time positions and targets are listed below.");
}
} // namespace ff_gui
