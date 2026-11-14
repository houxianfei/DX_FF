#include "ff_gui.h"
#include "ui_helpers.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>
namespace fs = std::filesystem;
namespace ff_gui {
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
    const auto config = service_.config();
    if (!config) return;
    for (int i = 0; i < service_.activeChannelCount(); i++) {
      const auto &ch = s.channels[i];
      for (size_t rodIndex = 0; rodIndex < ch.rods.size() &&
                                rodIndex < config->channels[i].rods.size(); rodIndex++)
        targets_.push_back(
            {{static_cast<ff::ChannelId>(i), config->channels[i].rods[rodIndex].id},
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
      const auto config = service_.config();
      if (!config) { notice_ = "Configuration is unavailable."; ImGui::EndDisabled(); return; }
      for (int i = 0; i < service_.activeChannelCount(); ++i) {
        const auto &channel = current.channels[i];
        for (size_t rod = 0; rod < channel.rods.size() &&
                               rod < config->channels[i].rods.size(); ++rod)
          preset.push_back({{static_cast<ff::ChannelId>(i),
                             config->channels[i].rods[rod].id},
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
        const auto c = service_.config();
        auto x = c ? ff::GuiPositionFiles::execute(service_,
            {c->machineId, c->configVersion, {}, preset}, false) :
            ff::Result<void>::failure(ff::ErrorCode::NotReady);
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
      const auto config = service_.config();
      auto loaded = config ? ff::GuiPositionFiles::load(path, *config) :
          ff::Result<ff::PositionFileData>::failure(ff::ErrorCode::NotReady);
      if (!loaded) {
        notice_ = "Position file rejected: " + loaded.message;
      } else {
        auto moved = ff::GuiPositionFiles::execute(service_, loaded.value, false);
        if (!moved) {
          notice_ = "Position file rejected: " + moved.message;
        } else {
          for (const auto &fileTarget : loaded.value.targets) {
            const auto mainTarget = std::find_if(targets_.begin(), targets_.end(),
              [&](const ff::RodTarget &entry) { return entry.address == fileTarget.address; });
            if (mainTarget == targets_.end()) targets_.push_back(fileTarget);
            else *mainTarget = fileTarget;
          }
          notice_ = "Position file loaded and executed";
          if (!loaded.value.summary.empty()) notice_ += ": " + loaded.value.summary;
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


} // namespace ff_gui

