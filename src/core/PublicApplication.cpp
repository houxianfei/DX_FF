#include "DX_FF.h"
#include "ff/FF.hpp"
#include "ff/Training.hpp"
#include "ff/PositionFiles.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <thread>

namespace FF {
namespace {
Result convert(const ff::Result<void>& r) {
  if (r) return {};
  ErrorCode code = ErrorCode::BackendFailure;
  switch (r.error) {
  case ff::ErrorCode::InvalidConfiguration: code=ErrorCode::InvalidConfiguration; break;
  case ff::ErrorCode::NotConnected: case ff::ErrorCode::NotReady: code=ErrorCode::NotOpen; break;
  case ff::ErrorCode::InvalidTarget: case ff::ErrorCode::RodNotFound:
  case ff::ErrorCode::DuplicateTarget: case ff::ErrorCode::LimitViolation: code=ErrorCode::InvalidTarget; break;
  case ff::ErrorCode::SafetyViolation: code=ErrorCode::SafetyViolation; break;
  case ff::ErrorCode::DeviceAlarm: code=ErrorCode::DeviceAlarm; break;
  case ff::ErrorCode::Communication: code=ErrorCode::Communication; break;
  case ff::ErrorCode::Timeout: code=ErrorCode::Timeout; break;
  case ff::ErrorCode::Busy: code=ErrorCode::Busy; break;
  default: break;
  }
  return {code,r.message};
}
Result fail(ErrorCode code, const std::string& message) { return {code,message}; }
bool validAddress(Address a) { return a.channel>=0 && a.channel<3 && a.rod>=0 && a.rod<=255; }
ff::RodTarget internal(Target t) {
  return {{static_cast<ff::ChannelId>(t.address.channel),static_cast<ff::RodId>(t.address.rod)},t.position};
}
Target external(ff::RodTarget t) {
  return {{static_cast<int>(t.address.channel),static_cast<int>(t.address.rod)},t.position};
}
std::vector<ff::RodTarget> internal(const std::vector<Target>& targets) {
  std::vector<ff::RodTarget> out; out.reserve(targets.size());
  for (auto t:targets) out.push_back(internal(t)); return out;
}
Result completeTargets(const std::vector<Target>& targets,
                       const ff::FFConfig& config, int activeChannels) {
  std::set<std::pair<int,int>> expected;
  for (int ch=0;ch<activeChannels;++ch)
    for (const auto& rod:config.channels[ch].rods)
      expected.insert({ch,rod.id});
  if (targets.size()!=expected.size())
    return fail(ErrorCode::InvalidTarget,"Complete target set required");
  for (const auto& target:targets) {
    const auto key=std::pair<int,int>{target.address.channel,target.address.rod};
    if (!expected.erase(key))
      return fail(ErrorCode::InvalidTarget,"Unknown or duplicate Rod target");
  }
  return {};
}
std::vector<ff::TrainingSample> internal(const std::vector<Sample>& samples) {
  std::vector<ff::TrainingSample> out; out.reserve(samples.size());
  for (auto s:samples) out.push_back({s.positionMm,s.energy}); return out;
}
ff::TrainingSession internal(const TrainingSession& session) {
  ff::TrainingSession raw;
  raw.source=static_cast<ff::TrainingSource>(session.source);
  raw.channel=static_cast<ff::ChannelId>(session.channel);
  raw.trainingRod=static_cast<ff::RodId>(session.trainingRod);
  raw.startMm=session.startMm; raw.endMm=session.endMm; raw.stepMm=session.stepMm;
  raw.valleySettings={session.valleySettings.smoothingPoints,
    session.valleySettings.minimumProminenceFraction,
    session.valleySettings.minimumDistanceMm,session.valleySettings.maximumCount};
  for (auto s:session.samples) raw.samples.push_back({s.positionMm,s.energy});
  raw.candidatesMm=session.candidatesMm;
  for (auto a:session.aims) raw.aims.push_back({
    {static_cast<ff::ChannelId>(a.address.channel),static_cast<ff::RodId>(a.address.rod)},
    a.requestedMm,a.quantizedMm,a.encoderAim,a.actualEncoder,a.assigned});
  for (auto r:session.rodStates) raw.rodStates.push_back({
    {static_cast<ff::ChannelId>(r.address.channel),static_cast<ff::RodId>(r.address.rod)},
    r.actualEncoder,r.targetEncoder});
  raw.occlusionSummary=session.occlusionSummary;
  raw.executionTargets=internal(session.executionTargets);
  raw.machineId=session.machineId; raw.configVersion=session.configVersion;
  raw.createdUnixSeconds=session.createdUnixSeconds; raw.complete=session.complete;
  return raw;
}
TrainingSession external(const ff::TrainingSession& raw) {
  TrainingSession out;
  out.source=static_cast<TrainingSource>(raw.source);
  out.channel=static_cast<int>(raw.channel); out.trainingRod=raw.trainingRod;
  out.startMm=raw.startMm; out.endMm=raw.endMm; out.stepMm=raw.stepMm;
  out.valleySettings={raw.valleySettings.smoothingPoints,
    raw.valleySettings.minimumProminenceFraction,
    raw.valleySettings.minimumDistanceMm,raw.valleySettings.maximumCount};
  for (auto s:raw.samples) out.samples.push_back({s.positionMm,s.energy});
  out.candidatesMm=raw.candidatesMm;
  for (auto a:raw.aims) out.aims.push_back({
    {static_cast<int>(a.address.channel),a.address.rod},a.requestedMm,
    a.quantizedMm,a.encoderAim,a.actualEncoder,a.assigned});
  for (auto r:raw.rodStates) out.rodStates.push_back({
    {static_cast<int>(r.address.channel),r.address.rod},r.actualEncoder,
    r.targetEncoder});
  out.occlusionSummary=raw.occlusionSummary;
  for (auto t:raw.executionTargets) out.executionTargets.push_back(external(t));
  out.machineId=raw.machineId; out.configVersion=raw.configVersion;
  out.createdUnixSeconds=raw.createdUnixSeconds; out.complete=raw.complete;
  return out;
}
const char* violationName(ff::SafetyViolation v) {
  switch(v) {
  case ff::SafetyViolation::None: return "None";
  case ff::SafetyViolation::CommunicationLost: return "CommunicationLost";
  case ff::SafetyViolation::PositionUnknown: return "PositionUnknown";
  case ff::SafetyViolation::DeviceAlarm: return "DeviceAlarm";
  case ff::SafetyViolation::HardLimitActive: return "HardLimitActive";
  case ff::SafetyViolation::SoftLimitViolation: return "SoftLimitViolation";
  case ff::SafetyViolation::ForbiddenRange: return "ForbiddenRange";
  case ff::SafetyViolation::RodInterference: return "RodInterference";
  case ff::SafetyViolation::InvalidTopology: return "InvalidTopology";
  case ff::SafetyViolation::StaleFeedback: return "StaleFeedback";
  case ff::SafetyViolation::ConfigurationInvalid: return "ConfigurationInvalid";
  case ff::SafetyViolation::ExternalUnsafe: return "ExternalUnsafe";
  }
  return "Unknown";
}
DeviceStatus external(const ff::FFStatus& s, const ff::FFConfig& config) {
  DeviceStatus out;
  switch(s.state) {
  case ff::FFState::Offline: out.state="Offline"; break;
  case ff::FFState::Ready: out.state="Ready"; break;
  case ff::FFState::Moving: out.state="Moving"; break;
  case ff::FFState::Fault: out.state="Fault"; break;
  case ff::FFState::EmergencyStopped: out.state="EmergencyStopped"; break;
  default: out.state="Other"; break;
  }
  for (int i=0;i<3;++i) {
    auto& c=out.channels[i]; c.channel=i; c.connected=s.channels[i].connected;
    c.safe=s.safety[i].state==ff::SafetyState::Safe;
    for (auto v:s.safety[i].violations) c.violations.push_back(violationName(v));
    for (size_t j=0;j<s.channels[i].rods.size() && j<config.channels[i].rods.size();++j) {
      const auto& r=s.channels[i].rods[j];
      c.rods.push_back({{i,config.channels[i].rods[j].id},r.actualPosition,r.targetPosition,
                        r.positionValid,r.enabled,r.alarm,r.alarmCode});
    }
  }
  return out;
}
} // namespace

struct ff_driver::Impl {
  ff::FFApplicationService service;
  std::optional<ff::FFConfig> config;
  ConfigSnapshot configSnapshot;
  DeviceStatus statusSnapshot;
  Result lastError;
  std::vector<RodPosition> currentPositions;
  StatusCallback callback;
  bool opened{false};
  std::mutex snapshotMutex;
};
ff_driver::ff_driver():impl_(std::make_unique<Impl>()),
  m_config(impl_->configSnapshot), m_status(impl_->statusSnapshot),
  m_last_error(impl_->lastError), m_current_positions(impl_->currentPositions) {
  impl_->statusSnapshot.state="Offline";
}
ff_driver::~ff_driver()=default;
void ff_driver::update_status() {
  if (impl_->opened && impl_->config) {
    auto state=external(impl_->service.status(),*impl_->config);
    std::lock_guard lock(impl_->snapshotMutex);
    impl_->statusSnapshot=std::move(state);
    update_current_positions();
  }
}
void ff_driver::update_current_positions() {
  impl_->currentPositions.clear();
  if (!impl_->opened || !impl_->config) return;
  for (int ch=0;ch<impl_->service.activeChannelCount();++ch) {
    const auto& rods=impl_->config->channels[ch].rods;
    for (size_t i=0;i<rods.size();++i) {
      const bool present=i<impl_->statusSnapshot.channels[ch].rods.size();
      const auto feedback=present?impl_->statusSnapshot.channels[ch].rods[i]:RodStatus{};
      impl_->currentPositions.push_back({{ch,rods[i].id},feedback.actual,
        rods[i].p0+rods[i].p1*feedback.actual,present && feedback.positionValid});
    }
  }
}
Result ff_driver::report(Result result) {
  { std::lock_guard lock(impl_->snapshotMutex); impl_->lastError=result; }
  update_status();
  return result;
}
Result ff_driver::open(const std::string& path, Mode mode,
                    const std::array<std::string,3>& ports, int activeChannels) {
  if (impl_->opened) return report(fail(ErrorCode::Busy,"Device is already open"));
  std::ifstream input(path,std::ios::binary);
  if (!input) return report(fail(ErrorCode::InvalidConfiguration,"Cannot read configuration"));
  std::string json((std::istreambuf_iterator<char>(input)),{});
  auto loaded=impl_->service.loadConfig(path,mode==Mode::Simulation,ports,activeChannels);
  if (!loaded) return report(convert(loaded));
  auto connected=impl_->service.connect();
  if (!connected) {
    impl_->service.disconnect(); return report(convert(connected));
  }
  impl_->config=impl_->service.config();
  impl_->configSnapshot={impl_->config->machineId,impl_->config->configVersion,std::move(json)};
  impl_->opened=true;
  impl_->service.setStatusCallback([this](const ff::FFStatus& s) {
    if (impl_->config) {
      std::lock_guard lock(impl_->snapshotMutex);
      impl_->statusSnapshot=external(s,*impl_->config);
      update_current_positions();
      if (impl_->callback) impl_->callback(impl_->statusSnapshot);
    }
  });
  update_status();
  return report({});
}
Result ff_driver::close() {
  if (!impl_->opened) return report({});
  auto r=impl_->service.disconnect();
  if(!r) return report(convert(r));
  impl_->opened=false; impl_->config.reset();
  { std::lock_guard lock(impl_->snapshotMutex);
    impl_->configSnapshot={}; impl_->statusSnapshot={};
    impl_->currentPositions.clear();
    impl_->statusSnapshot.state="Offline";
  }
  return report(convert(r));
}
Result ff_driver::refresh() {
  if (!impl_->opened) return report(fail(ErrorCode::NotOpen,"Device is not open"));
  return report(convert(impl_->service.refresh()));
}
Result ff_driver::refresh_rod(Address address) {
  if (!impl_->opened) return report(fail(ErrorCode::NotOpen,"Device is not open"));
  if (!validAddress(address) || address.channel >= impl_->service.activeChannelCount())
    return report(fail(ErrorCode::InvalidTarget,"Invalid Rod address"));
  return report(convert(impl_->service.refreshRod(internal(Target{address,0}).address)));
}
MachineInfo ff_driver::machine_info() const {
  MachineInfo out;
  if (!impl_->config) return out;
  out.activeChannels = impl_->service.activeChannelCount();
  for (int ch=0; ch<3; ++ch) {
    const auto& source=impl_->config->channels[ch];
    auto& target=out.channels[ch];
    target.channel=ch;
    target.trainingRod=source.trainingRod ? int(*source.trainingRod) : -1;
    for (const auto& rod:source.rods) {
      target.rods.push_back({{ch,rod.id},rod.name,rod.hardMin,rod.hardMax,
        rod.softMin,rod.softMax,
        rod.p0,rod.p1,rod.maxVelocity,rod.positionTolerance});
    }
    for (const auto& member:source.layerMembers)
      target.layerMembers.push_back({member.rod,member.layer,member.role,
                                     member.leftNeighbor,member.rightNeighbor});
    for (const auto& rule:source.clearanceRules)
      target.clearanceRules.push_back({rule.layer,rule.first,rule.second,
                                       rule.minimumClearanceMm});
  }
  return out;
}
void ff_driver::set_status_callback(StatusCallback cb) {
  impl_->callback=std::move(cb);
  report({});
}

ValueResult<PositionSet> ff_driver::load_positions(const std::string& path) {
  if (!impl_->opened) return {{},report(fail(ErrorCode::NotOpen,"Device is not open"))};
  auto loaded=ff::PositionFiles::load(path,*impl_->config);
  if(!loaded) return {{},report(convert(ff::Result<void>::failure(loaded.error,loaded.message)))};
  auto checked=ff::PositionFiles::validate(loaded.value,*impl_->config,
                                          impl_->service.activeChannelCount());
  if(!checked) return {{},report(convert(checked))};
  size_t expected=0;
  for(int ch=0;ch<impl_->service.activeChannelCount();++ch)
    expected+=impl_->config->channels[ch].rods.size();
  if(loaded.value.targets.size()!=expected)
    return {{},report(fail(ErrorCode::InvalidTarget,"Position file must include every active Rod"))};
  auto refreshed=refresh();
  if(!refreshed) return {{},refreshed};
  auto safe=impl_->service.validatePositionSet(loaded.value.targets);
  if(!safe) return {{},report(convert(safe))};
  PositionSet out;
  out.summary=loaded.value.summary;
  for(int ch=0;ch<impl_->service.activeChannelCount();++ch)
    for(const auto& rod:impl_->config->channels[ch].rods) {
      auto target=std::find_if(loaded.value.targets.begin(),loaded.value.targets.end(),
        [&](const auto& t){return int(t.address.channel)==ch && t.address.rod==rod.id;});
      if(target==loaded.value.targets.end())
        return {{},report(fail(ErrorCode::InvalidTarget,"Missing Rod target"))};
      out.targets.push_back(external(*target));
    }
  report({});
  return {std::move(out),{}};
}
Result ff_driver::save_positions(const std::string& path,const PositionSet& positions) {
  if (!impl_->opened) return report(fail(ErrorCode::NotOpen,"Device is not open"));
  auto complete=completeTargets(positions.targets,*impl_->config,
                                impl_->service.activeChannelCount());
  if (!complete) return report(complete);
  auto refreshed=refresh(); if (!refreshed) return refreshed;
  ff::PositionFileData file{impl_->config->machineId,impl_->config->configVersion,
                            positions.summary,internal(positions.targets)};
  auto checked=ff::PositionFiles::validate(file,*impl_->config,
                                           impl_->service.activeChannelCount());
  if (!checked) return report(convert(checked));
  auto safe=impl_->service.validatePositionSet(file.targets);
  if (!safe) return report(convert(safe));
  return report(convert(ff::PositionFiles::save(path,file)));
}
Result ff_driver::record_positions(const std::string& path,const std::string& summary) {
  auto refreshed=refresh(); if(!refreshed) return refreshed;
  auto config=impl_->service.config(); auto state=impl_->service.status();
  if(state.state!=ff::FFState::Ready)
    return report(fail(ErrorCode::SafetyViolation,"Device is not ready to record positions"));
  ff::PositionFileData file{config->machineId,config->configVersion,summary,{}};
  for(int i=0;i<impl_->service.activeChannelCount();++i) {
    if(state.safety[i].state!=ff::SafetyState::Safe)
      return report(fail(ErrorCode::SafetyViolation,"Current channel is unsafe"));
    for(size_t j=0;j<config->channels[i].rods.size();++j) {
      const auto& rod=config->channels[i].rods[j];
      const auto& feedback=state.channels[i].rods[j];
      if(!feedback.positionValid || feedback.alarm)
        return report(fail(ErrorCode::SafetyViolation,"Invalid Rod feedback"));
      file.targets.push_back({{static_cast<ff::ChannelId>(i),rod.id},
                              feedback.actualPosition});
    }
  }
  auto checked=ff::PositionFiles::validate(file,*impl_->config,
                                           impl_->service.activeChannelCount());
  if(!checked) return report(convert(checked));
  auto safe=impl_->service.validatePositionSet(file.targets);
  if(!safe) return report(convert(safe));
  return report(convert(ff::PositionFiles::save(path,file)));
}
Result ff_driver::move_rods(const std::vector<Target>& targets) {
  if (!impl_->opened) return report(fail(ErrorCode::NotOpen,"Device is not open"));
  auto complete=completeTargets(targets,*impl_->config,impl_->service.activeChannelCount());
  if (!complete) return report(complete);
  auto refreshed=refresh(); if (!refreshed) return refreshed;
  auto planned=impl_->service.previewPlan({internal(targets),false});
  if (!planned)
    return report(convert(ff::Result<void>::failure(planned.error,planned.message)));
  return report(convert(impl_->service.executePlan(planned.value)));
}
Result ff_driver::send_rods(const std::vector<Target>& targets) {
  if (!impl_->opened) return report(fail(ErrorCode::NotOpen,"Device is not open"));
  auto complete=completeTargets(targets,*impl_->config,impl_->service.activeChannelCount());
  if (!complete) return report(complete);
  return report(convert(impl_->service.setRodTargets(internal(targets))));
}
Result ff_driver::move_rod(Address address,std::int32_t encoder) {
  if(!impl_->opened) return report(fail(ErrorCode::NotOpen,"Device is not open"));
  if(!validAddress(address) || address.channel>=impl_->service.activeChannelCount())
    return report(fail(ErrorCode::InvalidTarget,"Invalid single Rod target"));
  auto config=impl_->service.rodConfig(internal(Target{address,encoder}).address);
  if(!config || encoder<config->softMin || encoder>config->softMax ||
     encoder<config->hardMin || encoder>config->hardMax)
    return report(fail(ErrorCode::InvalidTarget,"Rod target exceeds configured limits"));
  auto moved=impl_->service.moveRodAndWait(internal(Target{address,encoder}));
  if(!moved) return report(convert(moved));
  return report({});
}
Result ff_driver::send_rod(Address address,std::int32_t encoder) {
  if (!impl_->opened) return report(fail(ErrorCode::NotOpen,"Device is not open"));
  if (!validAddress(address) || address.channel>=impl_->service.activeChannelCount())
    return report(fail(ErrorCode::InvalidTarget,"Invalid single Rod target"));
  return report(convert(impl_->service.setRodTarget(internal(Target{address,encoder}))));
}
Result ff_driver::stop() { return impl_->opened ? report(convert(impl_->service.cancelMotion())) : report(fail(ErrorCode::NotOpen,"Device is not open")); }
Result ff_driver::emergency_stop() { return impl_->opened ? report(convert(impl_->service.emergencyStop())) : report(fail(ErrorCode::NotOpen,"Device is not open")); }
Result ff_driver::reset_faults() { return impl_->opened ? report(convert(impl_->service.resetFaults())) : report(fail(ErrorCode::NotOpen,"Device is not open")); }
ValueResult<Assignment> ff_driver::assign_training(int channel,
  const std::vector<double>& candidates,int excludedRod) {
  if(!impl_->opened) { auto r=report(fail(ErrorCode::NotOpen,"Device is not open")); return {{},r}; }
  if(channel<0 || channel>=impl_->service.activeChannelCount())
    { auto r=report(fail(ErrorCode::InvalidTarget,"Invalid channel")); return {{},r}; }
  auto refreshed=refresh(); if(!refreshed) return {{},refreshed};
  auto c=impl_->service.config(); auto s=impl_->service.status();
  std::optional<ff::RodId> excluded;
  if(excludedRod>=0 && excludedRod<=255) excluded=static_cast<ff::RodId>(excludedRod);
  auto r=ff::TrainingAnalysis::assign(*c,s,static_cast<ff::ChannelId>(channel),candidates,excluded);
  if(!r) { auto error=report(convert(ff::Result<void>::failure(r.error,r.message))); return {{},error}; }
  Assignment out; out.totalEncoderTravel=r.value.totalEncoderTravel;
  for(auto a:r.value.aims) out.aims.push_back({{channel,a.address.rod},a.requestedMm,a.quantizedMm,a.encoderAim,a.actualEncoder,a.assigned});
  report({});
  return {out,{}};
}
std::uint64_t ff_driver::simulated_energy(TrainingSource source,double p,double start,double end) {
  const auto result=ff::TrainingAnalysis::simulatedEnergy(
    static_cast<ff::TrainingSource>(source),p,start,end);
  report({}); return result;
}
double ff_driver::interpolate_energy(const std::vector<Sample>& samples,double p) {
  auto result=ff::TrainingAnalysis::interpolateEnergy(internal(samples),p); report({}); return result;
}
std::vector<double> ff_driver::find_valleys(const std::vector<Sample>& samples,const ValleySettings& s) {
  auto result=ff::TrainingAnalysis::findValleys(internal(samples),
    {s.smoothingPoints,s.minimumProminenceFraction,s.minimumDistanceMm,s.maximumCount});
  report({}); return result;
}
DragInterval ff_driver::drag_interval(const std::vector<double>& c,std::size_t i,double a,double start,double end,double w) {
  auto r=ff::TrainingAnalysis::dragInterval(c,i,a,start,end,w);
  report({});
  return {r.minimumMm,r.maximumMm,r.valid};
}
Result ff_driver::save_session(const std::string& path,const std::string& json) {
  // Validate the supplied document with the same parser used for loading.
  const auto temporary=path+".validate.tmp";
  { std::ofstream out(temporary,std::ios::binary); if(!out) return report(fail(ErrorCode::BackendFailure,"Cannot write session")); out<<json; }
  auto parsed=ff::TrainingAnalysis::loadSession(temporary);
  std::remove(temporary.c_str());
  if(!parsed) return report(convert(ff::Result<void>::failure(parsed.error,parsed.message)));
  return report(convert(ff::TrainingAnalysis::saveSession(path,parsed.value)));
}
ValueResult<std::string> ff_driver::load_session(const std::string& path) {
  auto parsed=ff::TrainingAnalysis::loadSession(path);
  if(!parsed) { auto error=report(convert(ff::Result<void>::failure(parsed.error,parsed.message))); return {{},error}; }
  std::ifstream input(path,std::ios::binary);
  report({});
  return {std::string((std::istreambuf_iterator<char>(input)),{}),{}};
}
Result ff_driver::save_training_session(const std::string& path,
                                         const TrainingSession& session) {
  if (session.channel<0 || session.channel>2 ||
      session.trainingRod<0 || session.trainingRod>255)
    return report(fail(ErrorCode::InvalidTarget,"Invalid training channel or Rod"));
  for (const auto& aim:session.aims)
    if (!validAddress(aim.address))
      return report(fail(ErrorCode::InvalidTarget,"Invalid training aim address"));
  for (const auto& state:session.rodStates)
    if (!validAddress(state.address))
      return report(fail(ErrorCode::InvalidTarget,"Invalid training Rod state address"));
  for (const auto& target:session.executionTargets)
    if (!validAddress(target.address))
      return report(fail(ErrorCode::InvalidTarget,"Invalid training target address"));
  auto raw=internal(session);
  const auto temporary=path+".validate.tmp";
  auto staged=ff::TrainingAnalysis::saveSession(temporary,raw);
  if (!staged) return report(convert(staged));
  auto checked=ff::TrainingAnalysis::loadSession(temporary);
  std::remove(temporary.c_str());
  if (!checked)
    return report(convert(ff::Result<void>::failure(checked.error,checked.message)));
  return report(convert(ff::TrainingAnalysis::saveSession(path,raw)));
}
ValueResult<TrainingSession> ff_driver::load_training_session(const std::string& path) {
  auto loaded=ff::TrainingAnalysis::loadSession(path);
  if (!loaded) {
    auto archive=ff::TrainingAnalysis::loadArchive(path);
    if (archive && archive.value.channels[archive.value.selectedChannel])
      loaded=ff::Result<ff::TrainingSession>::success(
        *archive.value.channels[archive.value.selectedChannel]);
  }
  if (!loaded) {
    auto r=report(convert(ff::Result<void>::failure(loaded.error,loaded.message)));
    return {{},r};
  }
  auto out=external(loaded.value);
  report({}); return {out,{}};
}
Result ff_driver::save_training_archive(const std::string& path,
                                        const TrainingArchive& archive) {
  if (archive.selectedChannel<0 || archive.selectedChannel>2)
    return report(fail(ErrorCode::InvalidTarget,"Invalid selected training channel"));
  ff::TrainingArchive raw;
  raw.machineId=archive.machineId;
  raw.configVersion=archive.configVersion;
  raw.createdUnixSeconds=archive.createdUnixSeconds;
  raw.selectedChannel=archive.selectedChannel;
  for (int ch=0;ch<3;++ch) {
    if (!archive.channels[ch]) continue;
    const auto& session=*archive.channels[ch];
    if (session.channel!=ch || session.trainingRod<0 || session.trainingRod>255)
      return report(fail(ErrorCode::InvalidTarget,"Invalid training channel entry"));
    for (const auto& aim:session.aims)
      if (!validAddress(aim.address) || aim.address.channel!=ch)
        return report(fail(ErrorCode::InvalidTarget,"Invalid training aim address"));
    for (const auto& state:session.rodStates)
      if (!validAddress(state.address) || state.address.channel!=ch)
        return report(fail(ErrorCode::InvalidTarget,"Invalid training Rod state address"));
    for (const auto& target:session.executionTargets)
      if (!validAddress(target.address) || target.address.channel!=ch)
        return report(fail(ErrorCode::InvalidTarget,"Invalid training target address"));
    raw.channels[ch]=internal(session);
  }
  const auto temporary=path+".validate.tmp";
  auto staged=ff::TrainingAnalysis::saveArchive(temporary,raw);
  if (!staged) return report(convert(staged));
  auto checked=ff::TrainingAnalysis::loadArchive(temporary);
  std::remove(temporary.c_str());
  if (!checked)
    return report(convert(ff::Result<void>::failure(checked.error,checked.message)));
  return report(convert(ff::TrainingAnalysis::saveArchive(path,raw)));
}
ValueResult<TrainingArchive> ff_driver::load_training_archive(const std::string& path) {
  auto loaded=ff::TrainingAnalysis::loadArchive(path);
  if (!loaded) {
    auto r=report(convert(ff::Result<void>::failure(loaded.error,loaded.message)));
    return {{},r};
  }
  TrainingArchive out;
  out.machineId=loaded.value.machineId;
  out.configVersion=loaded.value.configVersion;
  out.createdUnixSeconds=loaded.value.createdUnixSeconds;
  out.selectedChannel=loaded.value.selectedChannel;
  for (int ch=0;ch<3;++ch)
    if (loaded.value.channels[ch]) out.channels[ch]=external(*loaded.value.channels[ch]);
  report({});
  return {std::move(out),{}};
}
const char* ff_driver::version() const noexcept { return "1.0.1"; }
} // namespace FF
