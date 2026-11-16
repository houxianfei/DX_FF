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
  const auto assignment = ff::GuiTrainingAnalysis::assign(service_,
      *config, current, training_.channel, training_.candidatesMm,
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
    trainingNotice_ = assignment.value.message +
        ". Targets are ready to submit; SDK checks safety when motion starts.";
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
  // The simulation backend applies a goal synchronously.  Keep the hardware
  // state machine intact, but advance bounded synchronous simulation
  // transitions per frame.  Hardware scans use the shorter, user-visible scan
  // dwell instead of RodConfig::stableTime, which is intended for full plans.
  const bool fastSimulation = service_.simulation() &&
      (training_.source == ff::TrainingSource::Sim1 || training_.source == ff::TrainingSource::Sim2);
  const int budget = fastSimulation ? 48 : 1;
  for (int transition = 0; transition < budget && trainingScanning_; ++transition) {
    if (trainingMovePending_) {
      auto refreshed = service_.refreshRod(
          {training_.channel, training_.trainingRod});
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
        if (std::chrono::steady_clock::now() - *trainingStableSince_ <
            std::chrono::milliseconds(trainingSettleMs_)) break;
      }
      const double actualMm = it->p0 + it->p1 * feedback.actualPosition;
      training_.samples.push_back({actualMm, ff::GuiTrainingAnalysis::simulatedEnergy(service_, training_.source, actualMm, training_.startMm, training_.endMm)});
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
    ff::RodTarget moveTarget{{training_.channel, training_.trainingRod}, encoder};
    auto move = service_.setRodTarget(moveTarget);
    if (!move) { trainingScanning_ = false; trainingNotice_ = "Scan stopped: " + move.message; break; }
    trainingMovePending_ = true;
  }
}

void MainUi::training() {
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
  drawTrainingSettings(*config, channel, currentStatus, configuredTrainingRod,
                       trainingSettingsChanged);
  drawTrainingScanActions(*config, channel, configuredTrainingRod);
  drawTrainingCandidates(*config, channel, configuredTrainingRod,
                         layer2TrainingRodCount, trainingCandidateCapacityReached,
                         trainingSettingsChanged);
  drawTrainingResults(*config, channel, currentStatus);
  drawTrainingFiles(*config);
  if (trainingSettingsChanged)
    saveUiSettings();
  ImGui::End();
}


void MainUi::drawTrainingSettings(const ff::FFConfig& config, int channel, const ff::FFStatus& currentStatus, std::optional<ff::RodId> configuredTrainingRod, bool& trainingSettingsChanged) {
  constexpr float sourceWidth = 120.0f;
  constexpr float numberWidth = 108.0f;
  const auto& rods = config.channels[channel].rods;
  if (!ImGui::CollapsingHeader("Settings")) return;
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
  ImGui::SameLine();
  ImGui::SetNextItemWidth(numberWidth);
  if (ImGui::InputInt("Scan Settle (ms)", &trainingSettleMs_)) {
    trainingSettleMs_ = std::clamp(trainingSettleMs_, 0, 500);
    trainingSettingsChanged = true;
  }
  tooltip("After feedback reaches the configured position tolerance, wait this long before recording the scan sample. Default: 20 ms. This affects Training scans only, not normal motion-plan stability checks.");
  ImGui::SeparatorText("Valley Detection");
  ImGui::SetNextItemWidth(numberWidth); trainingSettingsChanged |= ImGui::InputInt("Smoothing", &training_.valleySettings.smoothingPoints);
  ImGui::SameLine(); ImGui::SetNextItemWidth(numberWidth); trainingSettingsChanged |= ImGui::InputDouble("Prominence", &training_.valleySettings.minimumProminenceFraction, .01, .1, "%.3f");
  ImGui::SameLine(); ImGui::SetNextItemWidth(numberWidth); trainingSettingsChanged |= ImGui::InputDouble("Min Distance (mm)", &training_.valleySettings.minimumDistanceMm, .1, 1, "%.3f");
  ImGui::SameLine(); ImGui::SetNextItemWidth(numberWidth); trainingSettingsChanged |= ImGui::InputInt("Max Valleys", &training_.valleySettings.maximumCount);
  if (trainingSettingsChanged) trainingForbiddenDirty_ = true;
}

void MainUi::drawTrainingScanActions(const ff::FFConfig& config, int channel, std::optional<ff::RodId> configuredTrainingRod) {
  constexpr float actionWidth = 112.0f;
  const auto& rods = config.channels[channel].rods;
  const bool canScan = configuredTrainingRod && !trainingScanning_ && training_.source != ff::TrainingSource::Real &&
                       training_.source != ff::TrainingSource::Loaded;
  ImGui::BeginDisabled(trainingScanning_);
  if (ImGui::Button("Initialize", ImVec2(actionWidth, 0))) initializeTraining();
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
    service_.cancelMotion(); trainingScanning_ = false; trainingMovePending_ = false; trainingStableSince_.reset(); training_.complete = false;
    trainingNotice_ = "Scan stopped; partial samples were retained.";
  }
  tooltip("Stop the active scan and retain all samples already collected.");
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Save Session", ImVec2(actionWidth, 0)))
    saveTrainingSession(config);
  tooltip("Save all three channel sessions to training_sessions with an automatic name.");
  ImGui::SameLine();
  if (ImGui::Button("Load Session", ImVec2(actionWidth, 0))) ImGuiFileDialog::Instance()->OpenDialog("TrainingLoad", "Load Training Session", ".json", {trainingDirectory_});
  tooltip("Load a previously saved Training session for viewing, editing, and retraining.");

}

void MainUi::initializeTraining() {
  resetTrainingMeasurement();
  if (!hasPresetSlot(0)) {
    trainingNotice_ = "Initialize unavailable: Position Slot 0 has not been saved.";
    return;
  }
  auto initialTargets = loadPreset(0);
  if (initialTargets.empty()) {
    trainingNotice_ = "Initialize failed: Position Slot 0 is invalid or empty.";
    return;
  }
  const auto config = service_.config();
  if (!config) {
    trainingNotice_ = "Initialize failed: no configuration is loaded.";
    return;
  }
  const auto result = ff::GuiPositionFiles::execute(
      service_, {config->machineId, config->configVersion, {}, initialTargets}, false);
  if (!result) {
    trainingNotice_ = "Initialize failed: " + result.message;
    return;
  }
  service_.refresh();
  targets_ = std::move(initialTargets);
  trainingNotice_ = "Moved to the positions saved in Position Slot 0.";
}

void MainUi::startTrainingScan(const ff::FFConfig& config,
                               const std::vector<ff::RodConfig>& rods) {
  if (training_.stepMm <= 0 || rods.empty()) {
    trainingNotice_ = "Step must be positive and the channel must contain a Rod.";
    return;
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
  training_.samples.clear();
  training_.candidatesMm.clear();
  training_.aims.clear();
  trainingExecutionTargets_.clear();
  trainingForbiddenDirty_ = true;
  training_.complete = false;
  training_.machineId = config.machineId;
  training_.configVersion = config.configVersion;
  training_.createdUnixSeconds = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  trainingNextMm_ = training_.startMm;
  trainingScanning_ = true;
  trainingMovePending_ = false;
  trainingStableSince_.reset();
  trainingNotice_ = "Scanning...";
}

void MainUi::drawTrainingCandidates(const ff::FFConfig& config, int channel, std::optional<ff::RodId> configuredTrainingRod, int layer2TrainingRodCount, bool trainingCandidateCapacityReached, bool& trainingSettingsChanged) {
  constexpr float actionWidth = 112.0f;
  const auto& rods = config.channels[channel].rods;
  if (training_.valleySettings.maximumCount <= 0) { training_.valleySettings.maximumCount = std::max(1, layer2TrainingRodCount); trainingSettingsChanged = true; }
  ImGui::BeginDisabled(training_.samples.size() < 3);
  if (ImGui::Button("Find Valleys", ImVec2(actionWidth, 0))) {
    const auto detected = ff::GuiTrainingAnalysis::findValleys(service_, training_.samples, training_.valleySettings);
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
}

void MainUi::drawTrainingResults(const ff::FFConfig& config, int channel, const ff::FFStatus& currentStatus) {
  constexpr float actionWidth = 112.0f;
  const auto& rods = config.channels[channel].rods;
  ImGui::BeginDisabled(training_.candidatesMm.empty());
  if (ImGui::Button("Plan", ImVec2(actionWidth, 0))) updateTrainingAssignment();
  tooltip("Assign candidate positions to Layer 2 Rods. Other eligible Layer 2 Rods receive the minimum safe evasion moves needed to preserve configured clearance.");
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
  for (int ch=0;ch<3;++ch) {
    const auto& session=trainingSessions_[ch];
    const bool hasRecord=ch==selected || session.complete ||
      !session.samples.empty() || !session.candidatesMm.empty() ||
      !session.aims.empty() || !session.rodStates.empty() ||
      !session.executionTargets.empty();
    if (!hasRecord) continue;
    if ((!session.machineId.empty() && session.machineId!=config.machineId) ||
        (!session.configVersion.empty() && session.configVersion!=config.configVersion)) {
      trainingNotice_="Save Session failed: a channel belongs to another machine or configuration.";
      return;
    }
    archive.channels[ch]=session;
  }
  try {
    const fs::path directory(trainingDirectory_);
    fs::create_directories(directory);
    fs::path path = directory / automaticTrainingFileName(training_);
    const auto stem=path.stem().string();
    for (int suffix=2;fs::exists(path);++suffix)
      path=directory/(stem+"_"+std::to_string(suffix)+".json");
    auto saved=ff::GuiTrainingAnalysis::saveArchive(service_,path.string(),archive);
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
        }
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

