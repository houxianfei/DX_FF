#include "gui_driver.hpp"

namespace ff {
namespace {
std::vector<::FF::Sample> samples(const std::vector<TrainingSample>& input) {
  std::vector<::FF::Sample> result;
  for (const auto& value:input) result.push_back({value.positionMm,value.energy});
  return result;
}
::FF::ValleySettings settings(const ValleyDetectionSettings& s) {
  return {s.smoothingPoints,s.minimumProminenceFraction,s.minimumDistanceMm,s.maximumCount};
}
TrainingSession fromPublic(const ::FF::TrainingSession& s) {
  TrainingSession out;
  out.source=static_cast<TrainingSource>(s.source);
  out.channel=static_cast<ChannelId>(s.channel); out.trainingRod=static_cast<RodId>(s.trainingRod);
  out.startMm=s.startMm; out.endMm=s.endMm; out.stepMm=s.stepMm;
  out.valleySettings={s.valleySettings.smoothingPoints,s.valleySettings.minimumProminenceFraction,
    s.valleySettings.minimumDistanceMm,s.valleySettings.maximumCount};
  for (auto v:s.samples) out.samples.push_back({v.positionMm,v.energy});
  out.candidatesMm=s.candidatesMm;
  for (auto v:s.aims) out.aims.push_back({
    {static_cast<ChannelId>(v.address.channel),static_cast<RodId>(v.address.rod)},
    v.requestedMm,v.quantizedMm,v.encoderAim,v.actualEncoder,v.assigned});
  for (auto v:s.rodStates) out.rodStates.push_back({
    {static_cast<ChannelId>(v.address.channel),static_cast<RodId>(v.address.rod)},
    v.actualEncoder,v.targetEncoder});
  out.occlusionSummary=s.occlusionSummary;
  for (auto v:s.executionTargets) out.executionTargets.push_back(privateTarget(v));
  out.machineId=s.machineId; out.configVersion=s.configVersion;
  out.createdUnixSeconds=s.createdUnixSeconds; out.complete=s.complete;
  return out;
}
::FF::TrainingSession toPublic(const TrainingSession& s) {
  ::FF::TrainingSession out;
  out.source=static_cast<::FF::TrainingSource>(s.source);
  out.channel=static_cast<int>(s.channel); out.trainingRod=s.trainingRod;
  out.startMm=s.startMm; out.endMm=s.endMm; out.stepMm=s.stepMm;
  out.valleySettings=settings(s.valleySettings);
  for (auto v:s.samples) out.samples.push_back({v.positionMm,v.energy});
  out.candidatesMm=s.candidatesMm;
  for (auto v:s.aims) out.aims.push_back({publicAddress(v.address),
    v.requestedMm,v.quantizedMm,v.encoderAim,v.actualEncoder,v.assigned});
  for (auto v:s.rodStates) out.rodStates.push_back({publicAddress(v.address),
    v.actualEncoder,v.targetEncoder});
  out.occlusionSummary=s.occlusionSummary;
  for (auto v:s.executionTargets) out.executionTargets.push_back(publicTarget(v));
  out.machineId=s.machineId; out.configVersion=s.configVersion;
  out.createdUnixSeconds=s.createdUnixSeconds; out.complete=s.complete;
  return out;
}
} // namespace

std::uint64_t GuiTrainingAnalysis::simulatedEnergy(TrainingSource source,double p,double start,double end) {
  return activeGuiDriver()->sdk().simulated_energy(static_cast<::FF::TrainingSource>(source),p,start,end);
}
double GuiTrainingAnalysis::interpolateEnergy(const std::vector<TrainingSample>& s,double p) {
  return activeGuiDriver()->sdk().interpolate_energy(samples(s),p);
}
std::vector<double> GuiTrainingAnalysis::findValleys(const std::vector<TrainingSample>& s,
    const ValleyDetectionSettings& v) {
  return activeGuiDriver()->sdk().find_valleys(samples(s),settings(v));
}
Result<RodAimAssignment> GuiTrainingAnalysis::assign(const FFConfig&,const FFStatus&,
    ChannelId channel,const std::vector<double>& candidates,std::optional<RodId> excluded) {
  auto r=activeGuiDriver()->sdk().assign_training(static_cast<int>(channel),candidates,
                                                   excluded?int(*excluded):-1);
  if (!r) return Result<RodAimAssignment>::failure(ErrorCode::InvalidTarget,r.result.message);
  RodAimAssignment out;
  out.totalEncoderTravel=r.value.totalEncoderTravel;
  out.message="Training assignment is valid";
  for (auto a:r.value.aims) out.aims.push_back({
    {static_cast<ChannelId>(a.address.channel),static_cast<RodId>(a.address.rod)},
    a.requestedMm,a.quantizedMm,a.encoderAim,a.actualEncoder,a.assigned});
  return Result<RodAimAssignment>::success(std::move(out));
}
Result<void> GuiTrainingAnalysis::validateAssignment(const FFConfig& c,const FFStatus& s,
    ChannelId channel,const std::vector<double>& candidates,std::optional<RodId> excluded) {
  auto r=assign(c,s,channel,candidates,excluded);
  if (!r) return Result<void>::failure(r.error,r.message);
  return Result<void>::success();
}
TrainingDragInterval GuiTrainingAnalysis::dragInterval(const std::vector<double>& candidates,
    std::size_t index,double anchor,double start,double end,double width) {
  auto r=activeGuiDriver()->sdk().drag_interval(candidates,index,anchor,start,end,width);
  return {r.minimumMm,r.maximumMm,r.valid};
}
Result<void> GuiTrainingAnalysis::saveSession(const std::string& path,const TrainingSession& session) {
  return privateResult(activeGuiDriver()->sdk().save_training_session(path,toPublic(session)));
}
Result<TrainingSession> GuiTrainingAnalysis::loadSession(const std::string& path) {
  auto r=activeGuiDriver()->sdk().load_training_session(path);
  if (!r) return Result<TrainingSession>::failure(ErrorCode::BackendFailure,r.result.message);
  return Result<TrainingSession>::success(fromPublic(r.value));
}
} // namespace ff

