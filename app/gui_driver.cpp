#include "gui_driver.hpp"

#include <algorithm>
#include <filesystem>
#include <windows.h>

namespace ff {
namespace {
GuiDriver* currentDriver = nullptr;
Result<void> fail(ErrorCode code, const std::string& message) {
  return Result<void>::failure(code, message);
}
} // namespace

GuiDriver::GuiDriver() { currentDriver = this; }
GuiDriver* activeGuiDriver() noexcept { return currentDriver; }

::FF::Address publicAddress(RodAddress a) {
  return {static_cast<int>(a.channel), static_cast<int>(a.rod)};
}
::FF::Target publicTarget(const RodTarget& t) {
  return {publicAddress(t.address), t.position, t.velocity};
}
RodTarget privateTarget(const ::FF::Target& t) {
  return {{static_cast<ChannelId>(t.address.channel),
           static_cast<RodId>(t.address.rod)}, t.position, t.velocity};
}
Result<void> privateResult(const ::FF::Result& r) {
  if (r) return Result<void>::success();
  ErrorCode code = ErrorCode::BackendFailure;
  switch (r.code) {
  case ::FF::ErrorCode::InvalidConfiguration: code=ErrorCode::InvalidConfiguration; break;
  case ::FF::ErrorCode::NotOpen: code=ErrorCode::NotConnected; break;
  case ::FF::ErrorCode::InvalidTarget: code=ErrorCode::InvalidTarget; break;
  case ::FF::ErrorCode::SafetyViolation: code=ErrorCode::SafetyViolation; break;
  case ::FF::ErrorCode::DeviceAlarm: code=ErrorCode::DeviceAlarm; break;
  case ::FF::ErrorCode::Communication: code=ErrorCode::Communication; break;
  case ::FF::ErrorCode::Timeout: code=ErrorCode::Timeout; break;
  case ::FF::ErrorCode::Busy: code=ErrorCode::Busy; break;
  default: break;
  }
  return fail(code,r.message);
}

Result<void> GuiDriver::loadConfig(const std::string& path, bool simulation,
    const std::array<std::string,3>& ports, int activeChannels) {
  if (connected_) return fail(ErrorCode::Busy,"Disconnect before loading configuration");
  auto loaded=FFConfigIO::load(path);
  if (!loaded) return fail(loaded.error,loaded.message);
  config_=std::move(loaded.value); configPath_=path;
  simulation_=simulation; ports_=ports; activeChannels_=activeChannels;
  return Result<void>::success();
}
Result<void> GuiDriver::applyConfig(const FFConfig& config, bool simulation,
    const std::array<std::string,3>& ports, int activeChannels) {
  if (connected_) return fail(ErrorCode::Busy,"Disconnect before applying configuration");
  auto checked=FFConfigIO::validate(config);
  if (!checked) return checked;
  const auto path=(std::filesystem::temp_directory_path() /
    ("ff_gui_applied_"+std::to_string(GetCurrentProcessId())+".json")).string();
  auto saved=FFConfigIO::save(path,config);
  if (!saved) return saved;
  config_=config; configPath_=path; simulation_=simulation;
  ports_=ports; activeChannels_=activeChannels;
  return Result<void>::success();
}
Result<void> GuiDriver::connect() {
  if (!config_) return fail(ErrorCode::InvalidConfiguration,"No configuration loaded");
  auto r=driver_.open(configPath_,
                      simulation_ ? ::FF::Mode::Simulation : ::FF::Mode::Hardware,
                      ports_,activeChannels_);
  connected_=static_cast<bool>(r); return privateResult(r);
}
Result<void> GuiDriver::disconnect() {
  auto r=driver_.close(); if (r) connected_=false; return privateResult(r);
}
Result<void> GuiDriver::refresh() { return privateResult(driver_.refresh()); }
Result<void> GuiDriver::refreshRod(RodAddress a) {
  return privateResult(driver_.refresh_rod(publicAddress(a)));
}
Result<void> GuiDriver::setRodTarget(const RodTarget& t) {
  return privateResult(driver_.send_rod(publicAddress(t.address),t.position,t.velocity));
}
Result<void> GuiDriver::setRodTargets(const std::vector<RodTarget>& targets) {
  auto refreshed=driver_.refresh();
  if (!refreshed) return privateResult(refreshed);
  std::vector<::FF::Target> complete;
  const auto machine=driver_.machine_info();
  for (const auto& current:driver_.m_current_positions) {
    double velocity=0;
    for (const auto& rod:machine.channels[current.address.channel].rods)
      if (rod.address.rod==current.address.rod) { velocity=rod.maxVelocity; break; }
    complete.push_back({current.address,current.encoder,velocity});
  }
  for (const auto& t:targets) {
    auto it=std::find_if(complete.begin(),complete.end(),[&](const auto& entry) {
      return entry.address.channel==static_cast<int>(t.address.channel) &&
             entry.address.rod==t.address.rod;
    });
    if (it==complete.end()) return fail(ErrorCode::InvalidTarget,"Unknown Rod target");
    *it=publicTarget(t);
  }
  auto submitted=driver_.set_target_positions(complete);
  if (!submitted) return privateResult(submitted);
  return privateResult(driver_.send_positions());
}
Result<void> GuiDriver::validateRodTargets(const std::vector<RodTarget>& targets) {
  std::vector<::FF::Target> values;
  for (const auto& t:targets) values.push_back(publicTarget(t));
  return privateResult(driver_.validate_target_positions(values));
}
Result<void> GuiDriver::cancelMotion() { return privateResult(driver_.stop()); }
Result<void> GuiDriver::emergencyStop() { return privateResult(driver_.emergency_stop()); }

FFStatus GuiDriver::status() const {
  FFStatus out;
  const auto& state=driver_.m_status;
  if (state.state=="Ready") out.state=FFState::Ready;
  else if (state.state=="Moving") out.state=FFState::Moving;
  else if (state.state=="Fault") out.state=FFState::Fault;
  else if (state.state=="EmergencyStopped") out.state=FFState::EmergencyStopped;
  else if (state.state=="Other") out.state=FFState::Degraded;
  else out.state=FFState::Offline;
  for (int ch=0;ch<3;++ch) {
    out.channels[ch].id=static_cast<ChannelId>(ch);
    out.channels[ch].connected=state.channels[ch].connected;
    out.safety[ch].channel=static_cast<ChannelId>(ch);
    out.safety[ch].state=state.channels[ch].safe?SafetyState::Safe:SafetyState::Unsafe;
    for (const auto& rod:state.channels[ch].rods) {
      RodStatus r;
      r.actualPosition=rod.actual; r.targetPosition=rod.target;
      r.positionValid=rod.positionValid; r.enabled=rod.enabled;
      r.alarm=rod.alarm; r.alarmCode=rod.alarmCode;
      r.state=rod.alarm?RodState::Fault:RodState::Idle;
      out.channels[ch].rods.push_back(r);
    }
  }
  return out;
}
std::optional<RodConfig> GuiDriver::rodConfig(RodAddress address) const {
  if (!config_ || static_cast<int>(address.channel)>=3) return std::nullopt;
  const auto& rods=config_->channels[static_cast<int>(address.channel)].rods;
  auto it=std::find_if(rods.begin(),rods.end(),[&](const auto& r){return r.id==address.rod;});
  if (it==rods.end()) return std::nullopt;
  return *it;
}
Result<PositionFileData> GuiPositionFiles::load(const std::string& path,const FFConfig& config) {
  auto* gui=activeGuiDriver();
  auto r=gui->sdk().load_positions(path);
  if (!r) return Result<PositionFileData>::failure(ErrorCode::InvalidTarget,r.message);
  PositionFileData out{config.machineId,config.configVersion,gui->sdk().m_position_summary,{}};
  for (const auto& value:gui->sdk().m_target_positions)
    out.targets.push_back(privateTarget({value.address,value.encoder,value.velocity}));
  return Result<PositionFileData>::success(std::move(out));
}
Result<void> GuiPositionFiles::save(const std::string& path,const PositionFileData& file) {
  (void)file;
  return privateResult(activeGuiDriver()->sdk().record_positions(path,file.summary));
}
Result<void> GuiPositionFiles::validate(const PositionFileData& file,const FFConfig& config,int count) {
  if (file.machineId!=config.machineId || file.configVersion!=config.configVersion)
    return fail(ErrorCode::InvalidTarget,"Position file identity mismatch");
  std::size_t expected=0;
  for (int ch=0;ch<count;++ch) expected+=config.channels[ch].rods.size();
  if (file.targets.size()!=expected) return fail(ErrorCode::InvalidTarget,"Incomplete position file");
  return Result<void>::success();
}
Result<void> GuiPositionFiles::execute(GuiDriver& service,const PositionFileData& file,bool) {
  return service.setRodTargets(file.targets);
}
} // namespace ff

