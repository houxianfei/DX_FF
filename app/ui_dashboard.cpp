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
void MainUi::drawChannel(int index, const ff::FFStatus &s) {
  auto &c = s.channels[index];
  ImGui::Text("CH%d: %zu configured device(s)", index + 1, c.rods.size());
}


void MainUi::draw() {
  if (initialization_.valid()) {
    if (initialization_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      const auto result=initialization_.get();
      notice_=result ? "Initialization complete." : "Initialization failed: "+result.message;
      trainingNotice_=notice_;
      service_.refresh();
      targetsInitialized_=false;
    }
  }
  if (showTraining_) {
    training();
    return;
  }
  if (ImGui::IsWindowCollapsed()) {
    if (showParameters_) parameters();
    return;
  }
  if (!configLoaded_ || !service_.connected()) {
    welcome();
  } else {
    dashboard();
  }
  if (showParameters_)
    parameters();
}
void MainUi::beginInitialize() {
  if (initialization_.valid()) return;
  if (!hasPresetSlot(0)) { notice_="Initialize failed: slot0 is missing."; trainingNotice_=notice_; return; }
  const auto path=fs::exists(presetPath(0)) ? presetPath(0) :
      (fs::path(presetDirectory_)/"slot_0.csv").string();
  statusBeforeInitialization_=service_.status();
  initialization_=std::async(std::launch::async,[this,path] {
    return service_.sdk().initialize(path);
  });
  notice_.clear();
  trainingNotice_.clear();
}
void MainUi::welcome() {
  static const char *modes[] = {"Simulation", "Real Hardware"};
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
  ImGui::TextUnformatted("Enabled Channels");
  for (int i=0;i<3;++i) {
    if (i) ImGui::SameLine();
    const auto label="CH"+std::to_string(i+1);
    bool selected=enabledChannels_[i];
    if (ImGui::Checkbox(label.c_str(),&selected)) {
      enabledChannels_[i]=selected;
      enabledChannelCount_=static_cast<int>(std::count(enabledChannels_.begin(),enabledChannels_.end(),true));
      if (!enabledChannels_[activeChannel_])
        for (int ch=0;ch<3;++ch) if (enabledChannels_[ch]) { activeChannel_=ch; break; }
      saveUiSettings();
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Scan Serial Ports"))
    scanSerialPorts();
  ImGui::TextDisabled("%zu serial port(s) detected", availablePorts_.size());
  for (int i = 0; i < 3; ++i) {
    if (!enabledChannels_[i]) continue;
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
    bool missingPort=false;
    for (int i = 0; i < 3; ++i) {
      if (!enabledChannels_[i]) continue;
      missingPort = missingPort || std::find(availablePorts_.begin(),availablePorts_.end(),serialPorts_[i])==availablePorts_.end();
      for (int j = i + 1; j < 3; ++j)
        duplicatePorts = duplicatePorts || (enabledChannels_[j] && serialPorts_[i] == serialPorts_[j]);
    }
    if (!enabledChannelCount_) notice_="Select at least one channel.";
    else if (!simulation_ && (duplicatePorts || missingPort)) {
      notice_ =
          "Select different detected COM ports for every enabled channel.";
    } else {
      auto loaded = service_.loadConfig(configPath_, simulation_, serialPorts_,
                                        enabledChannels_);
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
  const bool initializing=initialization_.valid();
  if (ImGui::BeginMenuBar()) {
    if (ImGui::BeginMenu("Tools")) {
      if (ImGui::MenuItem("FF_Config")) { showParameters_ = true; initializeParameters(); }
      if (ImGui::MenuItem("Training")) showTraining_ = true;
      ImGui::EndMenu();
    }
    ImGui::EndMenuBar();
  }
  const auto s = initializing ? statusBeforeInitialization_ : service_.status();
  ImGui::Text("RS485 is connected.  Mode: %s",
              simulation_ ? "Simulation" : "Real Hardware");
  ImGui::SameLine();
  ImGui::BeginDisabled(trainingScanning_ || initializing);
  if (ImGui::Button("close")) {
    auto x = service_.disconnect();
    if (!x)
      notice_ = x.message;
    targetsInitialized_ = false;
    ImGui::EndDisabled();
    return;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("EMERGENCY STOP")) {
    service_.emergencyStop();
    if (trainingScanning_) {
      trainingScanning_=false;
      trainingMovePending_=false;
      for (auto& session:trainingSessions_) session.complete=false;
      trainingNotice_="Scan stopped by emergency stop; completed samples were retained.";
    }
  }
  ImGui::BeginDisabled(trainingScanning_ || initializing);
  ImGui::SeparatorText("Control");
  ImGui::TextDisabled("Position controls apply to all enabled channels.");
  if (ImGui::Button(initializing ? "Initializing...##Initialize" : "Initialize##Initialize"))
    beginInitialize();
  ImGui::SameLine();
  ImGui::TextDisabled("Moves every enabled motor to Position Slot 0.");
  if (!targetsInitialized_ && service_.connected()) {
    targets_.clear();
    const auto config = service_.config();
    if (!config) { ImGui::EndDisabled(); return; }
    for (int i = 0; i < 3; i++) {
      if (!service_.enabledChannels()[i]) continue;
      const auto &ch = s.channels[i];
      for (size_t rodIndex = 0; rodIndex < ch.rods.size() &&
                                rodIndex < config->channels[i].rods.size(); rodIndex++)
        targets_.push_back(
            {{static_cast<ff::ChannelId>(i), config->channels[i].rods[rodIndex].id},
             ch.rods[rodIndex].actualPosition});
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
    for (int i = 0; i < 3; ++i)
      if (service_.enabledChannels()[i]) safe = safe && current.safety[i].state == ff::SafetyState::Safe;
    if (!safe) {
      notice_ = refreshed ? "Current positions are unsafe; preset was not saved."
                          : refreshed.message;
    } else {
      std::vector<ff::RodTarget> preset;
      const auto config = service_.config();
      if (!config) { notice_ = "Configuration is unavailable."; ImGui::EndDisabled(); ImGui::EndDisabled(); return; }
      for (int i = 0; i < 3; ++i) {
        if (!service_.enabledChannels()[i]) continue;
        const auto &channel = current.channels[i];
        for (size_t rod = 0; rod < channel.rods.size() &&
                               rod < config->channels[i].rods.size(); ++rod)
          preset.push_back({{static_cast<ff::ChannelId>(i),
                             config->channels[i].rods[rod].id},
                            channel.rods[rod].actualPosition});
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
      if (!preset) {
        notice_ = "Position preset rejected: " + preset.message;
      } else {
        auto x = ff::GuiPositionFiles::execute(service_,preset.value,false);
        if (x) {
          for (const auto& target:preset.value.targets) {
            auto existing=std::find_if(targets_.begin(),targets_.end(),
                [&](const ff::RodTarget& value){return value.address==target.address;});
            if (existing==targets_.end()) targets_.push_back(target);
            else *existing=target;
          }
          notice_ = slotLabel(selectedPresetSlot_) + " loaded.";
          if (!preset.value.ignoredChannels.empty()) {
            notice_ += " Ignored disabled ";
            for (size_t i=0;i<preset.value.ignoredChannels.size();++i) {
              if (i) notice_ += ", ";
              notice_ += "CH"+std::to_string(preset.value.ignoredChannels[i]+1);
            }
            notice_ += ".";
          }
        } else {
          notice_ = x.message + "; restored to the current safe positions.";
        }
      }
    }
  }
  ImGui::EndDisabled();
  if (ImGuiFileDialog::Instance()->Display("MainTrainingSlotLoad")) {
    if (ImGuiFileDialog::Instance()->IsOk() && !trainingScanning_) {
      const auto path = ImGuiFileDialog::Instance()->GetFilePathName();
      const auto config = service_.config();
      auto loaded = config ? ff::GuiPositionFiles::load(service_, path, *config) :
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
          if (!loaded.value.ignoredChannels.empty()) {
            notice_ += "; ignored disabled ";
            for (size_t index=0;index<loaded.value.ignoredChannels.size();++index) {
              if (index) notice_ += ", ";
              notice_ += "CH"+std::to_string(loaded.value.ignoredChannels[index]+1);
            }
          }
        }
      }
    }
    ImGuiFileDialog::Instance()->Close();
  }

  if (ImGui::BeginTabBar("channels")) {
    for (int i = 0; i < 3; i++) {
      if (!service_.enabledChannels()[i]) continue;
      const std::string label = "CH" + std::to_string(i + 1);
      if (ImGui::BeginTabItem(label.c_str())) {
        activeChannel_ = i;
        drawChannel(i, s);
        ImGui::EndTabItem();
      }
    }
    ImGui::EndTabBar();
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
  ImGui::EndDisabled();
  if (!notice_.empty())
    ImGui::TextColored(ImVec4(1, 0.5f, 0.2f, 1), "%s", notice_.c_str());
}


} // namespace ff_gui

