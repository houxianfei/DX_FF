#include "gui_driver.hpp"
#include <cmath>
#include <limits>

namespace ff {
Result<RodTarget> GuiTrainingAnalysis::scanTarget(
    const TrainingSession& session,const RodConfig& rod,int sampleIndex,int sampleCount) {
  if (sampleCount<2 || sampleCount>100000 || sampleIndex<0 ||
      sampleIndex>=sampleCount || session.trainingRod!=rod.id ||
      !std::isfinite(session.startMm) || !std::isfinite(session.endMm) ||
      session.startMm==session.endMm || !std::isfinite(rod.p0) ||
      !std::isfinite(rod.p1) || std::abs(rod.p1)<1e-12)
    return Result<RodTarget>::failure(ErrorCode::InvalidConfiguration,
        "Invalid training sample settings");
  const double mm=session.startMm+(session.endMm-session.startMm)*
      sampleIndex/(sampleCount-1);
  const double encoder=(mm-rod.p0)/rod.p1;
  if (!std::isfinite(encoder) ||
      encoder<std::numeric_limits<std::int32_t>::min() ||
      encoder>std::numeric_limits<std::int32_t>::max())
    return Result<RodTarget>::failure(ErrorCode::InvalidTarget,
        "Training sample encoder is out of range");
  return Result<RodTarget>::success({{session.channel,rod.id},
      static_cast<std::int32_t>(std::lround(encoder))});
}
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
  out.sampleCount=s.sampleCount;
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
  out.sampleCount=s.sampleCount;
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

std::uint64_t GuiTrainingAnalysis::simulatedEnergy(GuiDriver& driver,TrainingSource source,double p,double start,double end) {
  return driver.sdk().simulated_energy(static_cast<::FF::TrainingSource>(source),p,start,end);
}
double GuiTrainingAnalysis::interpolateEnergy(GuiDriver& driver,const std::vector<TrainingSample>& s,double p) {
  return driver.sdk().interpolate_energy(samples(s),p);
}
std::vector<double> GuiTrainingAnalysis::findValleys(GuiDriver& driver,const std::vector<TrainingSample>& s,
    const ValleyDetectionSettings& v) {
  return driver.sdk().find_valleys(samples(s),settings(v));
}
Result<RodAimAssignment> GuiTrainingAnalysis::assign(GuiDriver& driver,const FFConfig&,const FFStatus&,
    ChannelId channel,const std::vector<double>& candidates,std::optional<RodId> excluded) {
  auto r=driver.sdk().assign_training(static_cast<int>(channel),candidates,
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
Result<void> GuiTrainingAnalysis::validateAssignment(GuiDriver& driver,const FFConfig& c,const FFStatus& s,
    ChannelId channel,const std::vector<double>& candidates,std::optional<RodId> excluded) {
  auto r=assign(driver,c,s,channel,candidates,excluded);
  if (!r) return Result<void>::failure(r.error,r.message);
  return Result<void>::success();
}
TrainingDragInterval GuiTrainingAnalysis::dragInterval(GuiDriver& driver,const std::vector<double>& candidates,
    std::size_t index,double anchor,double start,double end,double width) {
  auto r=driver.sdk().drag_interval(candidates,index,anchor,start,end,width);
  return {r.minimumMm,r.maximumMm,r.valid};
}
Result<void> GuiTrainingAnalysis::saveSession(GuiDriver& driver,const std::string& path,const TrainingSession& session) {
  return privateResult(driver.sdk().save_training_session(path,toPublic(session)));
}
Result<TrainingSession> GuiTrainingAnalysis::loadSession(GuiDriver& driver,const std::string& path) {
  auto r=driver.sdk().load_training_session(path);
  if (!r) return Result<TrainingSession>::failure(ErrorCode::BackendFailure,r.result.message);
  return Result<TrainingSession>::success(fromPublic(r.value));
}
Result<void> GuiTrainingAnalysis::saveArchive(GuiDriver& driver,const std::string& path,
                                              const GuiTrainingArchive& archive) {
  ::FF::TrainingArchive value;
  value.machineId=archive.machineId;
  value.configVersion=archive.configVersion;
  value.createdUnixSeconds=archive.createdUnixSeconds;
  value.selectedChannel=archive.selectedChannel;
  value.enabledChannels=archive.enabledChannels;
  value.sampleCount=archive.sampleCount;
  for (int ch=0;ch<3;++ch)
    if (archive.channels[ch]) value.channels[ch]=toPublic(*archive.channels[ch]);
  return privateResult(driver.sdk().save_training_archive(path,value));
}
Result<GuiTrainingArchive> GuiTrainingAnalysis::loadArchive(GuiDriver& driver,
                                                            const std::string& path) {
  auto loaded=driver.sdk().load_training_archive(path);
  if (!loaded) return Result<GuiTrainingArchive>::failure(ErrorCode::BackendFailure,
                                                           loaded.result.message);
  GuiTrainingArchive out;
  out.machineId=loaded.value.machineId;
  out.configVersion=loaded.value.configVersion;
  out.createdUnixSeconds=loaded.value.createdUnixSeconds;
  out.selectedChannel=loaded.value.selectedChannel;
  out.enabledChannels=loaded.value.enabledChannels;
  out.sampleCount=loaded.value.sampleCount;
  for (int ch=0;ch<3;++ch)
    if (loaded.value.channels[ch]) out.channels[ch]=fromPublic(*loaded.value.channels[ch]);
  return Result<GuiTrainingArchive>::success(std::move(out));
}
} // namespace ff

