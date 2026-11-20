#include "ff_gui.h"
#include <ctime>
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
namespace {
constexpr float kSettingsSourceX=46.0f;
constexpr float kSettingsRodX=166.0f;
constexpr float kSettingsStartX=222.0f;
constexpr float kSettingsEndX=337.0f;
constexpr float kSettingsSmoothX=452.0f;
constexpr float kSettingsProminenceX=552.0f;
constexpr float kSettingsDistanceX=667.0f;
constexpr float kSettingsMaxX=797.0f;
void settingsColumn(float rowStart,float offset) {
  ImGui::SameLine(0.0f,0.0f);
  ImGui::SetCursorPosX(rowStart+offset);
}
std::string trainingSignature(const ff::TrainingSession& session,int settleMs) {
  std::ostringstream out;
  out << std::setprecision(17) << int(session.trainingRod) << ':'
      << int(session.source) << ':' << session.startMm << ':' << session.endMm
      << ':' << session.sampleCount << ':' << settleMs << ':'
      << session.valleySettings.smoothingPoints << ':'
      << session.valleySettings.minimumProminenceFraction << ':'
      << session.valleySettings.minimumDistanceMm << ':'
      << session.valleySettings.maximumCount << ':';
  for (double candidate:session.candidatesMm) out << candidate << ',';
  return out.str();
}
std::string automaticTrainingFileName(const ff::TrainingSession &session,
                                      std::array<bool,3> enabled) {
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
  name << "Training_";
  for (int ch=0;ch<3;++ch)
    if (enabled[ch]) name << "CH" << ch+1;
  name << "_SelectedCH" << (static_cast<int>(session.channel) + 1)
       << "_Occ" << session.candidatesMm.size()
       << "_" << assignedRange.str() << "_"
       << std::put_time(&localTime, "%Y%m%d_%H%M%S")
       << ".json";
  return name.str();
}
} // namespace

bool MainUi::candidateSetIsLegal(const std::vector<double> &candidates,
                                 std::string *reason) {
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
  const auto result = ff::GuiTrainingAnalysis::validateAssignment(service_,
      *config, service_.status(), training_.channel, candidates,
      training_.trainingRod);
  if (!result && reason) *reason = result.message;
  return static_cast<bool>(result);
}

void MainUi::resetTrainingMeasurement() {
  // Keep the operator's per-channel settings, but discard every value that
  // belongs to the previous acquisition / editing / training cycle.
  const int selected=static_cast<int>(training_.channel);
  if (selected>=0 && selected<3) trainingSessions_[selected]=training_;
  for (int ch=0;ch<3;++ch) {
    auto& session=trainingSessions_[ch];
    session.samples.clear();
    session.candidatesMm.clear();
    session.aims.clear();
    session.rodStates.clear();
    session.occlusionSummary.clear();
    session.executionTargets.clear();
    session.complete=false;
    session.machineId.clear();
    session.configVersion.clear();
    session.createdUnixSeconds=0;
  }
  if (selected>=0 && selected<3) training_=trainingSessions_[selected];
  trainingPlanSignatures_={};

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
  training_.executionTargets.clear();
  trainingPlanSignatures_[static_cast<int>(training_.channel)].clear();
  trainingEvasionTarget_.reset();
  const auto config = service_.config();
  if (!config) { trainingNotice_ = "No FF configuration is loaded."; return false; }
  const int channelIndex = static_cast<int>(training_.channel);
  const auto &channelConfig = config->channels[channelIndex];
  const auto current = service_.status();
  const auto assignment = ff::GuiTrainingAnalysis::assign(service_,
      *config, current, training_.channel, training_.candidatesMm,
      training_.trainingRod);
  if (!assignment) {
    training_.aims.clear(); training_.executionTargets.clear();
    trainingPlanSignatures_[channelIndex].clear();
    trainingSessions_[channelIndex]=training_;
    trainingNotice_ = assignment.message; return false;
  }
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
                       channelStatus.rods[index].actualPosition});
  }
  const auto setTarget = [&](const ff::RodTarget &replacement) {
    const auto target = std::find_if(targets.begin(), targets.end(),
        [&](const ff::RodTarget &entry) { return entry.address == replacement.address; });
    if (target != targets.end()) *target = replacement;
  };
  for (const auto &aim : training_.aims) {
    const auto rod = std::find_if(channelConfig.rods.begin(), channelConfig.rods.end(),
        [&](const ff::RodConfig &entry) { return entry.id == aim.address.rod; });
    if (rod != channelConfig.rods.end())
      setTarget({aim.address, aim.encoderAim});
  }
  training_.machineId = config->machineId;
  training_.configVersion = config->configVersion;
  const auto validated = service_.validateRodTargets(targets);
  if (validated) {
    trainingExecutionTargets_ = std::move(targets);
    training_.executionTargets=trainingExecutionTargets_;
    trainingPlanSignatures_[channelIndex]=trainingSignature(training_,trainingSettleMs_);
    trainingNotice_ = assignment.value.message +
        ". Targets are ready to submit; SDK checks safety when motion starts.";
  } else {
    training_.executionTargets.clear();
    trainingPlanSignatures_[channelIndex].clear();
    trainingNotice_ = assignment.value.message +
        ". Execute unavailable: " + validated.message;
  }
  trainingSessions_[channelIndex]=training_;
  return true;
}

void MainUi::tickTrainingScan() {
  if (!trainingScanning_) return;
  const auto config = service_.config();
  if (!config) { trainingScanning_=false; trainingNotice_="Scan stopped: configuration unavailable."; return; }
  const auto selected=static_cast<int>(training_.channel);
  trainingSessions_[selected]=training_;
  const auto fail=[&](const std::string& message) {
    service_.cancelMotion(); trainingScanning_=false; trainingMovePending_=false;
    for (int ch=0;ch<3;++ch) trainingSessions_[ch].complete=false;
    training_=trainingSessions_[selected]; trainingNotice_="Scan stopped: "+message;
  };
  const int budget=service_.simulation()?48:1;
  for (int pass=0;pass<budget && trainingScanning_;++pass) {
    if (!trainingMovePending_) {
      std::vector<ff::RodTarget> targets;
      for (int ch=0;ch<3;++ch) {
        if (!service_.enabledChannels()[ch]) continue;
        const auto& session=trainingSessions_[ch];
        const auto& rods=config->channels[ch].rods;
        const auto rod=std::find_if(rods.begin(),rods.end(),[&](const auto& r){return r.id==session.trainingRod;});
        if (rod==rods.end()) { fail("Training Rod missing."); return; }
        auto target=ff::GuiTrainingAnalysis::scanTarget(session,*rod,
            trainingSampleIndex_,trainingSampleCount_);
        if (!target) { fail(target.message); return; }
        targets.push_back(target.value);
        trainingChannelStable_[ch].reset();
      }
      auto sent=service_.setRodTargets(targets);
      if (!sent) { fail(sent.message); return; }
      trainingMovePending_=true;
      trainingStepStarted_=std::chrono::steady_clock::now();
      continue;
    }
    auto refreshed=service_.refresh();
    if (!refreshed) { fail(refreshed.message); return; }
    const auto status=service_.status();
    const auto now=std::chrono::steady_clock::now();
    bool allStable=true;
    std::array<double,3> actualPositions{};
    for (int ch=0;ch<3;++ch) {
      if (!service_.enabledChannels()[ch]) continue;
      const auto& session=trainingSessions_[ch];
      const auto& rods=config->channels[ch].rods;
      const auto rod=std::find_if(rods.begin(),rods.end(),[&](const auto& r){return r.id==session.trainingRod;});
      if (rod==rods.end()) { fail("Training Rod missing."); return; }
      const auto index=static_cast<size_t>(std::distance(rods.begin(),rod));
      if (index>=status.channels[ch].rods.size()) { fail("Rod feedback missing."); return; }
      const auto& feedback=status.channels[ch].rods[index];
      if (!feedback.positionValid || feedback.alarm) { fail("Rod feedback invalid or alarmed."); return; }
      auto target=ff::GuiTrainingAnalysis::scanTarget(session,*rod,
          trainingSampleIndex_,trainingSampleCount_);
      if (!target) { fail(target.message); return; }
      if (std::abs(feedback.actualPosition-target.value.position)>rod->positionTolerance)
        trainingChannelStable_[ch].reset();
      else if (!trainingChannelStable_[ch]) trainingChannelStable_[ch]=now;
      if (!trainingChannelStable_[ch] || now-*trainingChannelStable_[ch]<std::chrono::milliseconds(trainingSettleMs_))
        allStable=false;
      actualPositions[ch]=rod->p0+rod->p1*feedback.actualPosition;
      if ((!trainingChannelStable_[ch] ||
           now-*trainingChannelStable_[ch]<std::chrono::milliseconds(trainingSettleMs_)) &&
          now-trainingStepStarted_>rod->moveTimeout) { fail("Scan point timed out."); return; }
    }
    if (!allStable) break;
    std::array<ff::TrainingSample,3> round{};
    for (int ch=0;ch<3;++ch) {
      if (!service_.enabledChannels()[ch]) continue;
      const auto& session=trainingSessions_[ch];
      const double actual=actualPositions[ch];
      round[ch]={actual,ff::GuiTrainingAnalysis::simulatedEnergy(service_,
          session.source,actual,session.startMm,session.endMm)};
    }
    for (int ch=0;ch<3;++ch)
      if (service_.enabledChannels()[ch]) trainingSessions_[ch].samples.push_back(round[ch]);
    ++trainingSampleIndex_; trainingMovePending_=false;
    if (trainingSampleIndex_>=trainingSampleCount_) {
      trainingScanning_=false;
      for (int ch=0;ch<3;++ch)
        if (service_.enabledChannels()[ch]) trainingSessions_[ch].complete=true;
      trainingNotice_="Scan complete.";
    }
    training_=trainingSessions_[selected];
  }
}

void MainUi::training() {
  bool open = showTraining_;
  bool trainingSettingsChanged = false;
  const bool initializing=initialization_.valid();
  ImGuiWindowClass windowClass;
  windowClass.ViewportFlagsOverrideSet=ImGuiViewportFlags_NoAutoMerge;
  ImGui::SetNextWindowClass(&windowClass);
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->Pos,ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImGui::GetMainViewport()->Size,ImGuiCond_FirstUseEver);
  const bool expanded=ImGui::Begin("Training", &open);
  showTraining_ = open;
  if (!open && trainingScanning_) {
    service_.cancelMotion(); trainingScanning_ = false; trainingMovePending_ = false; trainingStableSince_.reset();
    training_.complete = false; trainingNotice_ = "Scan stopped when the window was closed; partial samples were retained.";
  }
  if (!expanded) {
    if (trainingScanning_) tickTrainingScan();
    ImGui::End(); return;
  }
  if (!service_.connected()) {
    ImGui::TextColored(ImVec4(1, .5f, .2f, 1), "Open FF before starting a training scan.");
    ImGui::End(); return;
  }
  tickTrainingScan();
  const auto config = service_.config();
  if (!config) { ImGui::Text("No configuration is loaded."); ImGui::End(); return; }
  int channel = static_cast<int>(training_.channel);
  if (!service_.enabledChannels()[channel]) {
    for (int ch=0;ch<3;++ch)
      if (service_.enabledChannels()[ch]) { channel=ch; training_=trainingSessions_[ch]; break; }
  }
  trainingSessions_[channel]=training_;
  ImGui::BeginDisabled(trainingScanning_ || initializing);
  if (ImGui::CollapsingHeader("Settings",ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextUnformatted("Samples");
    ImGui::SameLine(); ImGui::SetNextItemWidth(90.0f);
    if (ImGui::InputInt("##Samples", &trainingSampleCount_,0,0)) {
      trainingSampleCount_=std::clamp(trainingSampleCount_,2,100000);
      trainingSettingsChanged=true;
    }
    ImGui::SameLine(); ImGui::TextUnformatted("Scan Settle (ms)");
    ImGui::SameLine(); ImGui::SetNextItemWidth(90.0f);
    if (ImGui::InputInt("##Settle", &trainingSettleMs_,0,0)) {
      trainingSettleMs_=std::clamp(trainingSettleMs_,0,500);
      trainingSettingsChanged=true;
    }
    tooltip("Wait after every enabled Training Rod reaches its position tolerance before recording a sample. Default: 20 ms.");
    ImGui::Spacing();
    const float headerStart=ImGui::GetCursorPosX();
    ImGui::TextDisabled("CH");
    settingsColumn(headerStart,kSettingsSourceX); ImGui::TextDisabled("Data Source");
    settingsColumn(headerStart,kSettingsRodX); ImGui::TextDisabled("Rod");
    settingsColumn(headerStart,kSettingsStartX); ImGui::TextDisabled("Start (mm)");
    settingsColumn(headerStart,kSettingsEndX); ImGui::TextDisabled("End (mm)");
    settingsColumn(headerStart,kSettingsSmoothX); ImGui::TextDisabled("Smoothing");
    settingsColumn(headerStart,kSettingsProminenceX); ImGui::TextDisabled("Prominence");
    settingsColumn(headerStart,kSettingsDistanceX); ImGui::TextDisabled("Min Distance");
    settingsColumn(headerStart,kSettingsMaxX); ImGui::TextDisabled("Max Valleys");
    const auto currentStatus=initializing ? statusBeforeInitialization_ : service_.status();
    for (int ch=0;ch<3;++ch) {
      if (!service_.enabledChannels()[ch]) continue;
      ImGui::PushID(ch);
      auto saved=training_;
      training_=trainingSessions_[ch];
      drawTrainingSettings(*config,ch,currentStatus,config->channels[ch].trainingRod,
                           trainingSettingsChanged);
      training_.sampleCount=trainingSampleCount_;
      trainingSessions_[ch]=training_;
      training_=saved;
      ImGui::PopID();
    }
    ImGui::TextDisabled("Real camera acquisition is unavailable in this build.");
  }
  ImGui::EndDisabled();
  for (int ch=0;ch<3;++ch)
    if (service_.enabledChannels()[ch]) trainingSessions_[ch].sampleCount=trainingSampleCount_;
  training_=trainingSessions_[channel];
  trainingExecutionTargets_=training_.executionTargets;
  drawTrainingScanActions(*config,channel,config->channels[channel].trainingRod);
  ImGui::BeginDisabled(trainingScanning_ || initializing);
  const auto operationLabel=trainingOperationChannel_<0 ? std::string("All enabled channels") :
      "CH"+std::to_string(trainingOperationChannel_+1);
  ImGui::TextUnformatted("Analysis Target");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(220.0f);
  if (ImGui::BeginCombo("##AnalysisTarget",operationLabel.c_str())) {
    if (ImGui::Selectable("All enabled channels",trainingOperationChannel_<0)) trainingOperationChannel_=-1;
    for (int ch=0;ch<3;++ch) {
      if (!service_.enabledChannels()[ch]) continue;
      const auto label="CH"+std::to_string(ch+1);
      if (ImGui::Selectable(label.c_str(),trainingOperationChannel_==ch)) trainingOperationChannel_=ch;
    }
    ImGui::EndCombo();
  }
  const auto selectedForOperation=[&](int ch) {
    return service_.enabledChannels()[ch] && (trainingOperationChannel_<0 || trainingOperationChannel_==ch);
  };
  if (ImGui::Button("Find Valleys")) {
    trainingSessions_[channel]=training_;
    for (int ch=0;ch<3;++ch) {
      if (!selectedForOperation(ch)) continue;
      auto& session=trainingSessions_[ch];
      session.candidatesMm.clear(); session.aims.clear(); session.executionTargets.clear();
      trainingPlanSignatures_[ch].clear();
      const auto found=ff::GuiTrainingAnalysis::findValleys(service_,session.samples,session.valleySettings);
      for (double position:found) {
        const double left=std::min(session.startMm,session.endMm)+1.5;
        const double right=std::max(session.startMm,session.endMm)-1.5;
        if (position<left || position>right) continue;
        if (std::any_of(session.candidatesMm.begin(),session.candidatesMm.end(),
            [&](double previous){return std::abs(previous-position)<3.0;})) continue;
        session.candidatesMm.push_back(position);
      }
    }
    training_=trainingSessions_[channel]; trainingExecutionTargets_.clear();
    trainingForbiddenDirty_=true; trainingNotice_="Valley candidates updated.";
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Lines")) {
    trainingSessions_[channel]=training_;
    for (int ch=0;ch<3;++ch) {
      if (!selectedForOperation(ch)) continue;
      auto& session=trainingSessions_[ch];
      session.candidatesMm.clear();session.aims.clear();session.executionTargets.clear();
      trainingPlanSignatures_[ch].clear();
    }
    training_=trainingSessions_[channel];trainingExecutionTargets_.clear();
    trainingForbiddenDirty_=true;trainingNotice_="Candidate lines cleared.";
  }
  ImGui::SameLine();
  if (ImGui::Button("Plan")) {
    trainingSessions_[channel]=training_;
    const auto displayed=channel;
    bool any=false,success=true;
    for (int ch=0;ch<3;++ch) {
      if (!selectedForOperation(ch) || trainingSessions_[ch].candidatesMm.empty()) continue;
      any=true;training_=trainingSessions_[ch];
      if (!updateTrainingAssignment() || training_.executionTargets.empty()) {
        success=false;break;
      }
    }
    training_=trainingSessions_[displayed];
    trainingExecutionTargets_=training_.executionTargets;
    if (!any) trainingNotice_="No candidate lines to plan.";
    else if (success) trainingNotice_="Selected channels planned; Execute checks all enabled channels.";
  }
  ImGui::SameLine();
  bool anyCandidates=false;
  for (int ch=0;ch<3;++ch)
    if (service_.enabledChannels()[ch] && !trainingSessions_[ch].candidatesMm.empty())
      anyCandidates=true;
  ImGui::BeginDisabled(!anyCandidates);
  if (ImGui::Button("Execute")) {
    trainingSessions_[channel]=training_;
    if (!service_.sdk().m_status.initialized) trainingNotice_="Initialize from slot0 before Execute.";
    else {
      std::vector<ff::RodTarget> combined;
      bool valid=true,any=false;
      for (int ch=0;ch<3;++ch) {
        if (!service_.enabledChannels()[ch]) continue;
        const auto& session=trainingSessions_[ch];
        if (session.candidatesMm.empty()) continue;
        any=true;
        if (session.executionTargets.empty() ||
            trainingPlanSignatures_[ch]!=trainingSignature(session,trainingSettleMs_)) {
          valid=false;trainingNotice_="CH"+std::to_string(ch+1)+" needs a current Plan.";break;
        }
        combined.insert(combined.end(),session.executionTargets.begin(),session.executionTargets.end());
      }
      if (!any) trainingNotice_="No candidate lines to execute.";
      else if (valid) {
        auto result=service_.setRodTargets(combined);
        if (!result) trainingNotice_="Execute failed: "+result.message;
        else {
          service_.refresh();targetsInitialized_=false;
          trainingNotice_="Training targets sent for all planned channels.";
        }
      }
    }
  }
  ImGui::EndDisabled();
  ImGui::EndDisabled();
  if (ImGui::BeginTabBar("TrainingChannels")) {
    for (int candidate = 0; candidate < 3; ++candidate) {
      if (!service_.enabledChannels()[candidate]) continue;
      const std::string label = "CH" + std::to_string(candidate + 1);
      if (!ImGui::BeginTabItem(label.c_str())) continue;
      if (channel != candidate) {
      trainingSessions_[channel] = training_;
      channel = candidate;
      training_ = trainingSessions_[channel];
      training_.channel = static_cast<ff::ChannelId>(channel);
      trainingExecutionTargets_=training_.executionTargets;
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
  const auto currentStatus = initializing ? statusBeforeInitialization_ : service_.status();
  ImGui::BeginDisabled(trainingScanning_ || initializing);
  drawTrainingCandidates(*config, channel, configuredTrainingRod,
                         layer2TrainingRodCount, trainingCandidateCapacityReached,
                         trainingSettingsChanged);
  drawTrainingResults(*config, channel, currentStatus);
  ImGui::EndDisabled();
  ImGui::BeginDisabled(initializing);
  drawTrainingFiles(*config);
  ImGui::EndDisabled();
  if (trainingSettingsChanged)
    saveUiSettings();
  ImGui::End();
}


void MainUi::drawTrainingSettings(const ff::FFConfig& config, int channel, const ff::FFStatus& currentStatus, std::optional<ff::RodId> configuredTrainingRod, bool& trainingSettingsChanged) {
  const auto& rods = config.channels[channel].rods;
  const char *sources[] = {"Sim1", "Sim2", "Real", "Loaded"};
  int source = static_cast<int>(training_.source);
  const float rowStart=ImGui::GetCursorPosX();
  ImGui::Text("CH%d",channel+1);
  settingsColumn(rowStart,kSettingsSourceX);
  ImGui::SetNextItemWidth(110.0f);
  if (ImGui::BeginCombo("##Source",sources[std::clamp(source,0,3)])) {
    for (int option=0;option<IM_ARRAYSIZE(sources);++option) {
      ImGui::BeginDisabled(option==static_cast<int>(ff::TrainingSource::Real));
      if (ImGui::Selectable(sources[option],source==option)) {
        training_.source=static_cast<ff::TrainingSource>(option);
        training_.aims.clear();
        training_.executionTargets.clear();
        trainingExecutionTargets_.clear();
        trainingEvasionTarget_.reset();
        trainingSettingsChanged=true;
      }
      ImGui::EndDisabled();
    }
    ImGui::EndCombo();
  }
  settingsColumn(rowStart,kSettingsRodX);
  if (configuredTrainingRod) {
    ImGui::TextDisabled("R%d",int(*configuredTrainingRod));
  } else {
    ImGui::TextColored(ImVec4(1.0f,0.5f,0.2f,1.0f),"Missing");
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
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Current: %d (%.4f mm)\nTarget: %d (%.4f mm)",
            feedback.actualPosition,selectedRod->p0+selectedRod->p1*feedback.actualPosition,
            feedback.targetPosition,selectedRod->p0+selectedRod->p1*feedback.targetPosition);
    }
  }
  settingsColumn(rowStart,kSettingsStartX); ImGui::SetNextItemWidth(105.0f);
  trainingSettingsChanged |= ImGui::InputDouble("##Start",&training_.startMm,0,0,"%.4f");
  settingsColumn(rowStart,kSettingsEndX); ImGui::SetNextItemWidth(105.0f);
  trainingSettingsChanged |= ImGui::InputDouble("##End",&training_.endMm,0,0,"%.4f");
  settingsColumn(rowStart,kSettingsSmoothX); ImGui::SetNextItemWidth(90.0f);
  trainingSettingsChanged |= ImGui::InputInt("##Smooth",&training_.valleySettings.smoothingPoints,0,0);
  settingsColumn(rowStart,kSettingsProminenceX); ImGui::SetNextItemWidth(105.0f);
  trainingSettingsChanged |= ImGui::InputDouble("##Prominence",&training_.valleySettings.minimumProminenceFraction,0,0,"%.3f");
  settingsColumn(rowStart,kSettingsDistanceX); ImGui::SetNextItemWidth(120.0f);
  trainingSettingsChanged |= ImGui::InputDouble("##Distance",&training_.valleySettings.minimumDistanceMm,0,0,"%.3f");
  settingsColumn(rowStart,kSettingsMaxX); ImGui::SetNextItemWidth(90.0f);
  trainingSettingsChanged |= ImGui::InputInt("##MaxValleys",&training_.valleySettings.maximumCount,0,0);
  if (trainingSettingsChanged) trainingForbiddenDirty_ = true;
}

void MainUi::drawTrainingScanActions(const ff::FFConfig& config, int channel, std::optional<ff::RodId> configuredTrainingRod) {
  constexpr float actionWidth = 112.0f;
  const auto& rods = config.channels[channel].rods;
  const bool initializing=initialization_.valid();
  const bool canScan = configuredTrainingRod && !trainingScanning_ && !initializing && training_.source != ff::TrainingSource::Real &&
                       training_.source != ff::TrainingSource::Loaded;
  ImGui::BeginDisabled(trainingScanning_ || initializing);
  if (ImGui::Button(initializing ? "Initializing...##Initialize" : "Initialize##Initialize",
                    ImVec2(actionWidth, 0))) initializeTraining();
  tooltip("Move all Rods to the positions saved in Main UI Position Slot 0.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!canScan);
  if (ImGui::Button("Start Scan", ImVec2(actionWidth, 0)))
    startTrainingScan(config, rods);
  tooltip("Move the selected Training Rod through the configured range and capture one energy sample at every step.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!trainingScanning_);
  if (ImGui::Button("Stop Scan", ImVec2(actionWidth, 0))) {
    service_.cancelMotion();
    trainingScanning_ = false; trainingMovePending_ = false;
    trainingStableSince_.reset(); training_.complete = false;
    trainingNotice_ = "Scan stopped; partial samples were retained.";
  }
  tooltip("Stop the active scan and retain all samples already collected.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("EMERGENCY STOP")) {
    auto stopped=service_.emergencyStop();
    trainingScanning_=false;
    trainingMovePending_=false;
    trainingStableSince_.reset();
    training_.complete=false;
    for (auto& session:trainingSessions_) session.complete=false;
    trainingNotice_=stopped ? "Emergency stop; completed samples were retained."
                            : "Emergency stop failed: "+stopped.message;
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(initializing);
  if (ImGui::Button("Save Session", ImVec2(actionWidth, 0)))
    saveTrainingSession(config);
  tooltip("Save the sessions for all enabled channels to training_sessions with an automatic name.");
  ImGui::SameLine();
  if (ImGui::Button("Load Session", ImVec2(actionWidth, 0))) ImGuiFileDialog::Instance()->OpenDialog("TrainingLoad", "Load Training Session", ".json", {trainingDirectory_});
  tooltip("Load a previously saved Training session for viewing, editing, and retraining.");
  ImGui::EndDisabled();

}

void MainUi::initializeTraining() {
  if (!hasPresetSlot(0)) { beginInitialize(); return; }
  resetTrainingMeasurement();
  beginInitialize();
}

void MainUi::startTrainingScan(const ff::FFConfig& config,
                               const std::vector<ff::RodConfig>& rods) {
  if (!service_.sdk().m_status.initialized) {
    trainingNotice_="Initialize from complete slot0 before scanning.";
    return;
  }
  trainingSessions_[static_cast<int>(training_.channel)]=training_;
  if (trainingSampleCount_<2 || trainingSampleCount_>100000) {
    trainingNotice_="Sample count must be 2..100000."; return;
  }
  for (int ch=0;ch<3;++ch) {
    if (!service_.enabledChannels()[ch]) continue;
    auto& session=trainingSessions_[ch];
    session.sampleCount=trainingSampleCount_;
    const auto& channel=config.channels[ch];
    if (!channel.trainingRod || session.source==ff::TrainingSource::Real ||
        session.source==ff::TrainingSource::Loaded) {
      trainingNotice_="CH"+std::to_string(ch+1)+" has no available Training Rod or source."; return;
    }
    session.trainingRod=*channel.trainingRod;
    const auto rod=std::find_if(channel.rods.begin(),channel.rods.end(),
        [&](const auto& r){return r.id==session.trainingRod;});
    if (rod==channel.rods.end() || !std::isfinite(session.startMm) ||
        !std::isfinite(session.endMm) || session.startMm==session.endMm ||
        !std::isfinite(rod->p0) || !std::isfinite(rod->p1) || std::abs(rod->p1)<1e-12) {
      trainingNotice_="CH"+std::to_string(ch+1)+" scan settings are invalid."; return;
    }
    const double a=rod->p0+rod->p1*rod->softMin;
    const double b=rod->p0+rod->p1*rod->softMax;
    if (std::min(session.startMm,session.endMm)<std::min(a,b) ||
        std::max(session.startMm,session.endMm)>std::max(a,b)) {
      trainingNotice_="CH"+std::to_string(ch+1)+" scan range exceeds soft limits."; return;
    }
    for (int sample=0;sample<trainingSampleCount_;++sample) {
      auto target=ff::GuiTrainingAnalysis::scanTarget(session,*rod,sample,
          trainingSampleCount_);
      if (!target) { trainingNotice_=target.message; return; }
      const auto encoder=target.value.position;
      if (encoder<rod->softMin || encoder>rod->softMax ||
          encoder<rod->hardMin || encoder>rod->hardMax ||
          std::any_of(rod->forbiddenRanges.begin(),rod->forbiddenRanges.end(),
              [encoder](const auto& range){return encoder>=range.first && encoder<=range.second;})) {
        trainingNotice_="CH"+std::to_string(ch+1)+" sample "+
            std::to_string(sample+1)+" violates a Rod limit or forbidden range.";
        return;
      }
    }
  }
  if (std::abs(training_.startMm - training_.endMm) < 1e-9) {
    trainingNotice_ = "Start and end must be different to produce a curve.";
    return;
  }
  const auto selected = std::find_if(rods.begin(), rods.end(),
      [this](const ff::RodConfig& rod) { return rod.id == training_.trainingRod; });
  if (selected == rods.end() || std::abs(selected->p1) < 1e-12) {
    trainingNotice_ = "Training Rod has invalid P0/P1 calibration.";
    return;
  }
  const double first = selected->p0 + selected->p1 * selected->softMin;
  const double last = selected->p0 + selected->p1 * selected->softMax;
  const double lo = std::min(first, last);
  const double hi = std::max(first, last);
  if (training_.startMm < lo || training_.startMm > hi ||
      training_.endMm < lo || training_.endMm > hi) {
    trainingNotice_ = "Start and end must be within the selected Rod soft range.";
    return;
  }
  auto fresh=service_.refresh();
  if (!fresh) { trainingNotice_="Scan preflight failed: "+fresh.message; return; }
  for (int sample=0;sample<trainingSampleCount_;++sample) {
    std::vector<::FF::Target> targets;
    for (int ch=0;ch<3;++ch) {
      if (!service_.enabledChannels()[ch]) continue;
      const auto& session=trainingSessions_[ch];
      const auto& configured=config.channels[ch].rods;
      const auto rod=std::find_if(configured.begin(),configured.end(),
          [&](const auto& value){return value.id==session.trainingRod;});
      auto target=ff::GuiTrainingAnalysis::scanTarget(session,*rod,sample,
          trainingSampleCount_);
      if (!target) { trainingNotice_=target.message; return; }
      targets.push_back({{ch,int(session.trainingRod)},target.value.position});
    }
    auto valid=service_.sdk().validate_rods(targets);
    if (!valid) {
      trainingNotice_="Scan preflight failed at sample "+std::to_string(sample+1)+
          ": "+valid.message;
      return;
    }
  }
  training_.samples.clear();
  training_.candidatesMm.clear();
  training_.aims.clear();
  training_.executionTargets.clear();
  trainingExecutionTargets_.clear();
  trainingForbiddenDirty_ = true;
  training_.complete = false;
  training_.machineId = config.machineId;
  training_.configVersion = config.configVersion;
  training_.createdUnixSeconds = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  trainingSessions_[static_cast<int>(training_.channel)]=training_;
  for (int ch=0;ch<3;++ch) {
    if (!service_.enabledChannels()[ch] || ch==static_cast<int>(training_.channel)) continue;
    auto& session=trainingSessions_[ch];
    session.samples.clear(); session.candidatesMm.clear(); session.aims.clear();
    session.executionTargets.clear(); session.complete=false;
    session.machineId=config.machineId; session.configVersion=config.configVersion;
    session.createdUnixSeconds=training_.createdUnixSeconds;
  }
  trainingPlanSignatures_={};
  trainingDraggingCandidate_=-1;
  trainingSampleIndex_=0;
  trainingMovePending_=false;
  trainingChannelStable_={};
  trainingNextMm_ = training_.startMm;
  trainingScanning_ = true;
  trainingMovePending_ = false;
  trainingStableSince_.reset();
  trainingNotice_ = "Scanning...";
}

void MainUi::drawTrainingCandidates(const ff::FFConfig& config, int channel, std::optional<ff::RodId> configuredTrainingRod, int layer2TrainingRodCount, bool trainingCandidateCapacityReached, bool& trainingSettingsChanged) {
  const auto& rods = config.channels[channel].rods;
  if (training_.valleySettings.maximumCount <= 0) { training_.valleySettings.maximumCount = std::max(1, layer2TrainingRodCount); trainingSettingsChanged = true; }

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
      if (!trainingScanning_ && ImPlot::IsPlotHovered()) {
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
          training_.executionTargets.clear();
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
              training_.aims.clear(); training_.executionTargets.clear();
              trainingExecutionTargets_.clear(); trainingForbiddenDirty_ = true;
            }
            else trainingNotice_ = "Candidate rejected: " + reason;
          }
        }
      }
      if (!trainingScanning_ && trainingDraggingCandidate_ >= 0 &&
          ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        const auto movingIndex =
            static_cast<size_t>(trainingDraggingCandidate_);
        if (movingIndex < training_.candidatesMm.size()) {
          const double proposed = ImPlot::GetPlotMousePos().x;
          const auto interval = ff::GuiTrainingAnalysis::dragInterval(service_,
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
          training_.executionTargets.clear();
          trainingExecutionTargets_.clear();
          trainingForbiddenDirty_ = true;
        }
      }
      if (!trainingScanning_ && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
          trainingDraggingCandidate_ >= 0) {
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
}

void MainUi::drawTrainingResults(const ff::FFConfig& config, int channel, const ff::FFStatus& currentStatus) {
  const auto& rods = config.channels[channel].rods;
  ImGui::SeparatorText("Layer 2 Rod Positions and Training Aims");
  if (ImGui::BeginTable("TrainingAims", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Rod"); ImGui::TableSetupColumn("Current mm"); ImGui::TableSetupColumn("Train Aim mm"); ImGui::TableSetupColumn("Safe / Evasion mm"); ImGui::TableSetupColumn("Assignment"); ImGui::TableHeadersRow();
    for (size_t index = 0; index < rods.size(); ++index) {
      const auto &rodConfig = rods[index];
      bool inLayer2 = false, irisRelated = false;
      for (const auto &member : config.channels[channel].layerMembers) {
        if (member.rod != rodConfig.id) continue;
        inLayer2 = inLayer2 || member.layer == 2;
        irisRelated = irisRelated || member.layer != 2;
      }
      if (!inLayer2 || irisRelated) continue;
      const ff::RodStatus *feedback = index < currentStatus.channels[channel].rods.size() ? &currentStatus.channels[channel].rods[index] : nullptr;
      const auto aim = std::find_if(training_.aims.begin(), training_.aims.end(), [&](const ff::TrainingAim &entry) { return entry.address.rod == rodConfig.id; });
      const bool isEvasionRod = aim != training_.aims.end() && !aim->assigned;
      ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Text("Rod %d", int(rodConfig.id));
      ImGui::TableSetColumnIndex(1); if (feedback) ImGui::Text("%.4f", rodConfig.p0 + rodConfig.p1 * feedback->actualPosition); else ImGui::TextUnformatted("--");
      ImGui::TableSetColumnIndex(2); if (aim != training_.aims.end() && aim->assigned) ImGui::Text("%.4f", aim->quantizedMm); else ImGui::TextUnformatted("--");
      ImGui::TableSetColumnIndex(3);
      if (isEvasionRod) ImGui::Text("%.4f", aim->quantizedMm);
      else ImGui::TextUnformatted("--");
      ImGui::TableSetColumnIndex(4);
      if (isEvasionRod) ImGui::TextUnformatted(
          trainingExecutionTargets_.empty() ? "evasion / aim only"
                                            : "evasion / ready");
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
}

void MainUi::saveTrainingSession(const ff::FFConfig& config) {
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
  const auto &savedChannel = config.channels[static_cast<int>(training_.channel)];
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
  const int selected=static_cast<int>(training_.channel);
  trainingSessions_[selected]=training_;
  ff::GuiTrainingArchive archive;
  archive.machineId=config.machineId;
  archive.configVersion=config.configVersion;
  archive.createdUnixSeconds=static_cast<std::int64_t>(std::time(nullptr));
  archive.selectedChannel=selected;
  archive.enabledChannels=service_.enabledChannels();
  archive.sampleCount=trainingSampleCount_;
  for (int ch=0;ch<3;++ch) {
    if (!archive.enabledChannels[ch]) continue;
    auto& session=trainingSessions_[ch];
    if ((!session.machineId.empty() && session.machineId!=config.machineId) ||
        (!session.configVersion.empty() && session.configVersion!=config.configVersion)) {
      trainingNotice_="Save Session failed: CH"+std::to_string(ch+1)+" belongs to another configuration.";
      return;
    }
    session.sampleCount=trainingSampleCount_;
    session.channel=static_cast<ff::ChannelId>(ch);
    session.machineId=config.machineId;
    session.configVersion=config.configVersion;
    session.rodStates.clear();
    for (size_t rod=0;rod<config.channels[ch].rods.size() &&
         rod<current.channels[ch].rods.size();++rod)
      session.rodStates.push_back({{static_cast<ff::ChannelId>(ch),config.channels[ch].rods[rod].id},
          current.channels[ch].rods[rod].actualPosition,
          current.channels[ch].rods[rod].targetPosition});
    archive.channels[ch]=session;
  }
  try {
    const fs::path directory(trainingDirectory_);
    fs::create_directories(directory);
    fs::path path = directory / automaticTrainingFileName(training_,archive.enabledChannels);
    const auto stem=path.stem().string();
    for (int suffix=2;fs::exists(path);++suffix)
      path=directory/(stem+"_"+std::to_string(suffix)+".json");
    auto temporary=path.string()+".tmp";
    auto saved=ff::GuiTrainingAnalysis::saveArchive(service_,temporary,archive);
    if (saved) {
      try { fs::rename(temporary,path); }
      catch (const fs::filesystem_error& error) {
        fs::remove(temporary);
        saved=ff::Result<void>::failure(ff::ErrorCode::BackendFailure,error.what());
      }
    }
    trainingNotice_=saved ? "Training session saved: "+path.string()
                          : "Save Session failed: "+saved.message;
  } catch (const fs::filesystem_error& error) {
    trainingNotice_="Save Session failed: "+std::string(error.what());
  }
}

void MainUi::drawTrainingFiles(const ff::FFConfig& config) {
  if (ImGuiFileDialog::Instance()->Display("TrainingLoad")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      auto loaded = ff::GuiTrainingAnalysis::loadArchive(service_, ImGuiFileDialog::Instance()->GetFilePathName());
      if (loaded) {
        for (int ch=0;ch<3;++ch) {
          trainingSessions_[ch]=loaded.value.channels[ch].value_or(ff::TrainingSession{});
          trainingSessions_[ch].channel=static_cast<ff::ChannelId>(ch);
          trainingSessions_[ch].executionTargets.clear();
          trainingSessions_[ch].aims.clear();
        }
        trainingPlanSignatures_={};
        trainingSampleCount_=std::clamp(loaded.value.sampleCount,2,100000);
        const int loadedChannel=loaded.value.selectedChannel;
        training_=trainingSessions_[loadedChannel];
        training_.source=ff::TrainingSource::Loaded;
        trainingExecutionTargets_.clear();
        trainingEvasionTarget_.reset();
        trainingForbiddenDirty_ = true;
        trainingSessions_[loadedChannel] = training_;
        if (loaded.value.machineId != config.machineId || loaded.value.configVersion != config.configVersion) { training_.aims.clear(); trainingNotice_ = "Session loaded for viewing; machine/config differs, so Train is required before Execute."; }
        else trainingNotice_ = "Training session loaded; click Train to validate current hardware state.";
      } else trainingNotice_ = loaded.message;
    }
    ImGuiFileDialog::Instance()->Close();
  }
}

} // namespace ff_gui

