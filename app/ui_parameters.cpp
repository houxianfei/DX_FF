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

  drawParameterSystem();
  if (ImGui::BeginTabBar("ParameterChannels")) {
    for (int channelIndex = 0; channelIndex < 3; ++channelIndex) {
      const std::string label = "CH" + std::to_string(channelIndex + 1);
      if (!ImGui::BeginTabItem(label.c_str())) continue;
      auto& channel = parameterDraft_.channels[channelIndex];
      drawParameterController(channel);
      drawParameterTraining(channel);
      if (ImGui::BeginTabBar("ConfigSections")) {
        drawParameterRods(channel, label);
        drawParameterLayers(channel);
        drawParameterClearance(channel);
        ImGui::EndTabBar();
      }
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  if (ImGuiFileDialog::Instance()->Display("FFConfigLoad")) { if (ImGuiFileDialog::Instance()->IsOk()) loadParameterFile(ImGuiFileDialog::Instance()->GetFilePathName()); ImGuiFileDialog::Instance()->Close(); }
  if (ImGuiFileDialog::Instance()->Display("FFConfigSave")) { if (ImGuiFileDialog::Instance()->IsOk()) saveParameterFile(ImGuiFileDialog::Instance()->GetFilePathName()); ImGuiFileDialog::Instance()->Close(); }
  if (ImGui::BeginPopupModal("Discard unsaved configuration?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) { ImGui::Text("Save changes before continuing?"); if (ImGui::Button("Save")) { if (saveParameterFile(configPath_) && pendingLoadPath_ == "__open_dialog__") ImGuiFileDialog::Instance()->OpenDialog("FFConfigLoad", "Load FF Configuration", ".json", {}); pendingLoadPath_.clear(); ImGui::CloseCurrentPopup(); } ImGui::SameLine(); if (ImGui::Button("Discard")) { parameterDraft_ = parameterOriginal_; parametersDirty_ = false; if (pendingLoadPath_ == "__open_dialog__") ImGuiFileDialog::Instance()->OpenDialog("FFConfigLoad", "Load FF Configuration", ".json", {}); else showParameters_ = false; pendingLoadPath_.clear(); ImGui::CloseCurrentPopup(); } ImGui::SameLine(); if (ImGui::Button("Cancel")) { pendingLoadPath_.clear(); ImGui::CloseCurrentPopup(); } ImGui::EndPopup(); }
  ImGui::End();
}
void MainUi::drawParameterSystem() {
  if (ImGui::CollapsingHeader("System", ImGuiTreeNodeFlags_DefaultOpen)) {
    bool changed = inputString("Machine ID", parameterDraft_.machineId);
    changed |= inputString("Config Version", parameterDraft_.configVersion);
    if (changed) markParametersDirty();
    int maxRods = static_cast<int>(parameterDraft_.maxActiveRodCount);
    int expiration = static_cast<int>(parameterDraft_.statusExpiration.count());
    changed = ImGui::InputInt("Max Active Rod Count", &maxRods);
    changed |= ImGui::InputInt("Status Expiration (ms)", &expiration);
    if (changed) {
      parameterDraft_.maxActiveRodCount = std::max(0, maxRods);
      parameterDraft_.statusExpiration = std::chrono::milliseconds(std::max(0, expiration)); markParametersDirty();
    }
    ff::RodConfig *shared = nullptr;
    for (auto &channel : parameterDraft_.channels)
      if (!channel.rods.empty()) { shared = &channel.rods.front(); break; }
    if (shared) {
      ImGui::SeparatorText("All Rod Motor Parameters");
      int velocity = static_cast<int>(shared->maxVelocity);
      int acceleration = static_cast<int>(shared->maxAcceleration);
      int deceleration = static_cast<int>(shared->maxDeceleration);
      int tolerance = shared->positionTolerance;
      int stable = static_cast<int>(shared->stableTime.count());
      int timeout = static_cast<int>(shared->moveTimeout.count());
      bool motionChanged = ImGui::InputInt("Speed Limit (raw 1-1023)", &velocity);
      motionChanged |= ImGui::InputInt("Acceleration (raw 0-255)", &acceleration);
      motionChanged |= ImGui::InputInt("Deceleration (raw 0-255)", &deceleration);
      ImGui::SeparatorText("All Rod Completion Settings");
      motionChanged |= ImGui::InputInt("Position Tolerance (count)", &tolerance);
      motionChanged |= ImGui::InputInt("Stable Time (ms)", &stable);
      motionChanged |= ImGui::InputInt("Move Timeout (ms)", &timeout);
      if (motionChanged) {
        for (auto &channel : parameterDraft_.channels)
          for (auto &rod : channel.rods) {
            rod.maxVelocity = velocity;
            rod.maxAcceleration = acceleration;
            rod.maxDeceleration = deceleration;
            rod.positionTolerance = tolerance;
            rod.stableTime = std::chrono::milliseconds(std::max(0, stable));
            rod.moveTimeout = std::chrono::milliseconds(std::max(0, timeout));
          }
        markParametersDirty();
      }
      ImGui::TextDisabled("Motor values are checked and written at connection when different.");
    }
  }

}

void MainUi::drawParameterController(ff::FFChannelConfig& channel) {
    if (ImGui::CollapsingHeader("Controller", ImGuiTreeNodeFlags_DefaultOpen)) {
      auto &controller = channel.controller;
      bool changed = inputString("Controller Name", controller.name);
      changed |= inputString("Config Serial Port", controller.serial.portName);
      if (changed) markParametersDirty();
      int baud = static_cast<int>(controller.serial.baudRate), timeout = static_cast<int>(controller.serial.timeout.count());
      int poll = static_cast<int>(controller.pollPeriod.count()), retry = static_cast<int>(controller.retryCount);
      bool valuesChanged = ImGui::InputInt("Baud Rate", &baud);
      valuesChanged |= ImGui::InputInt("Timeout (ms)", &timeout);
      valuesChanged |= ImGui::InputInt("Poll Period (ms)", &poll);
      valuesChanged |= ImGui::InputInt("Retry Count", &retry);
      if (valuesChanged) {
        controller.serial.baudRate = std::max(0, baud); controller.serial.timeout = std::chrono::milliseconds(std::max(0, timeout));
        controller.pollPeriod = std::chrono::milliseconds(std::max(0, poll)); controller.retryCount = std::max(0, retry); markParametersDirty();
      }
      ImGui::TextDisabled("Connection settings on the Welcome page override this port at runtime.");
    }

}

void MainUi::drawParameterTraining(ff::FFChannelConfig& channel) {
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

}

void MainUi::drawParameterRods(ff::FFChannelConfig& channel, const std::string& label) {
  if (!ImGui::BeginTabItem("Rods")) return;
  if (ImGui::BeginTable("Rods", 8, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp, ImVec2(0, 190))) {
    ImGui::TableSetupScrollFreeze(2, 1);
    ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 45); ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Soft Min (count)"); ImGui::TableSetupColumn("Soft Max (count)"); ImGui::TableSetupColumn("Safe (count)"); ImGui::TableSetupColumn("P0 (mm)"); ImGui::TableSetupColumn("P1 (mm/count)"); ImGui::TableSetupColumn("Speed Raw"); ImGui::TableHeadersRow();
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
    if (id < 256) { ff::RodConfig rod; rod.id = static_cast<ff::RodId>(id); rod.name = label + "-R" + std::to_string(id); rod.hardMax = rod.softMax = 4095; rod.p1 = 1;
      for (const auto &other : parameterDraft_.channels) if (!other.rods.empty()) { const auto &shared = other.rods.front(); rod.maxVelocity = shared.maxVelocity; rod.maxAcceleration = shared.maxAcceleration; rod.maxDeceleration = shared.maxDeceleration; rod.positionTolerance = shared.positionTolerance; rod.stableTime = shared.stableTime; rod.moveTimeout = shared.moveTimeout; break; }
      channel.rods.push_back(rod); selectedRod_ = int(channel.rods.size()) - 1; markParametersDirty(); }
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
    bool changed = inputString("Rod Name", rod.name);
    changed |= ImGui::InputInt("Hard Min (count)", &rod.hardMin);
    changed |= ImGui::InputInt("Hard Max (count)", &rod.hardMax);
    changed |= ImGui::InputInt("Soft Min (count)", &rod.softMin);
    changed |= ImGui::InputInt("Soft Max (count)", &rod.softMax);
    changed |= ImGui::InputInt("Zero Position (count)", &rod.zeroPosition);
    changed |= ImGui::InputInt("Safe Position (count)", &rod.safePosition);
    changed |= ImGui::Checkbox("Safe Position Valid", &rod.safePositionValid);
    changed |= ImGui::InputDouble("P0 (mm)", &rod.p0);
    changed |= ImGui::InputDouble("P1 (mm/count)", &rod.p1);
    changed |= ImGui::InputInt("Inward Direction", &rod.inwardDirection);
    if (changed) markParametersDirty();
  }
  ImGui::EndTabItem();
}

void MainUi::drawParameterLayers(ff::FFChannelConfig& channel) {
  if (!ImGui::BeginTabItem("Layer Memberships")) return;
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
    bool changed = ImGui::InputInt("Member Rod", &rod);
    changed |= ImGui::InputInt("Member Layer", &m.layer);
    changed |= ImGui::InputInt("Member Role", &m.role);
    changed |= ImGui::InputInt("Member Left Neighbor", &m.leftNeighbor);
    changed |= ImGui::InputInt("Member Right Neighbor", &m.rightNeighbor);
    if (changed) { m.rod = static_cast<ff::RodId>(std::clamp(rod, 0, 255)); markParametersDirty(); }
  }
  ImGui::EndTabItem();
}

void MainUi::drawParameterClearance(ff::FFChannelConfig& channel) {
  if (!ImGui::BeginTabItem("Clearance Rules")) return;
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
    bool changed = ImGui::InputInt("Rule Layer", &r.layer);
    changed |= ImGui::InputInt("Rule Rod A", &first);
    changed |= ImGui::InputInt("Rule Rod B", &second);
    changed |= ImGui::InputDouble("Rule Minimum Clearance (mm)", &r.minimumClearanceMm);
    if (changed) { r.first = static_cast<ff::RodId>(std::clamp(first, 0, 255)); r.second = static_cast<ff::RodId>(std::clamp(second, 0, 255)); markParametersDirty(); }
  }
  ImGui::EndTabItem();
}

} // namespace ff_gui
