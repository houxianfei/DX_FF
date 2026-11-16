#include "ff/FF.hpp"
#include "ff/Training.hpp"
#include "ff/PositionFiles.hpp"
#include <cassert>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
using namespace ff;
class FailingOpenBackend final : public IControllerBackend {
public:
  explicit FailingOpenBackend(bool fail):fail_(fail),delegate_(makeSimulationBackend()) {}
  Result<void> open(const SerialConfig& c) override {
    return fail_ ? Result<void>::failure(ErrorCode::Communication,"injected open failure") : delegate_->open(c);
  }
  Result<void> close() override { return delegate_->close(); }
  bool isOpen() const noexcept override { return delegate_->isOpen(); }
  Result<void> configureRods(const std::vector<RodConfig>& c) override { return delegate_->configureRods(c); }
  Result<void> setGoal(RodId id,int32_t p) override { return delegate_->setGoal(id,p); }
  Result<void> setGoalsSynchronous(const std::vector<RodTarget>& t) override { return delegate_->setGoalsSynchronous(t); }
  Result<void> stop(RodId id) override { return delegate_->stop(id); }
  Result<void> stopAll() override { return delegate_->stopAll(); }
  Result<void> enable(RodId id,bool b) override { return delegate_->enable(id,b); }
  Result<void> home(RodId id) override { return delegate_->home(id); }
  Result<RodStatus> readStatus(RodId id) override { return delegate_->readStatus(id); }
  Result<void> resetAlarm(RodId id) override { return delegate_->resetAlarm(id); }
private:
  bool fail_;
  std::unique_ptr<IControllerBackend> delegate_;
};
struct ProbeState {
  std::atomic<int> commands{0};
  bool invalidFeedback{false};
  bool alarm{false};
};
class ProbeBackend final : public IControllerBackend {
public:
  explicit ProbeBackend(std::shared_ptr<ProbeState> state)
      : state_(std::move(state)), delegate_(makeSimulationBackend()) {}
  Result<void> open(const SerialConfig& c) override { return delegate_->open(c); }
  Result<void> close() override { return delegate_->close(); }
  bool isOpen() const noexcept override { return delegate_->isOpen(); }
  Result<void> configureRods(const std::vector<RodConfig>& c) override { return delegate_->configureRods(c); }
  Result<void> setGoal(RodId id,int32_t p) override {
    ++state_->commands; return delegate_->setGoal(id,p);
  }
  Result<void> setGoalsSynchronous(const std::vector<RodTarget>& t) override {
    ++state_->commands; return delegate_->setGoalsSynchronous(t);
  }
  Result<void> stop(RodId id) override { return delegate_->stop(id); }
  Result<void> stopAll() override { return delegate_->stopAll(); }
  Result<void> enable(RodId id,bool b) override { return delegate_->enable(id,b); }
  Result<void> home(RodId id) override { return delegate_->home(id); }
  Result<RodStatus> readStatus(RodId id) override {
    auto result=delegate_->readStatus(id);
    if(result && id==0) {
      result.value.positionValid=!state_->invalidFeedback;
      result.value.alarm=state_->alarm;
    }
    return result;
  }
  Result<void> resetAlarm(RodId id) override { return delegate_->resetAlarm(id); }
private:
  std::shared_ptr<ProbeState> state_;
  std::unique_ptr<IControllerBackend> delegate_;
};
static FFConfig cfg() {
  FFConfig c;
  c.machineId = "test";
  c.configVersion = "v1";
  c.maxActiveRodCount = 3;
  for (int i = 0; i < 3; i++) {
    auto &ch = c.channels[i];
    ch.id = static_cast<ChannelId>(i);
    ch.controller.serial.portName = "COM" + std::to_string(i + 1);
    RodConfig r;
    r.id = 0;
    r.hardMin = 0;
    r.hardMax = 100;
    r.softMin = 10;
    r.softMax = 90;
    r.safePosition = 80;
    r.safePositionValid = true;
    r.maxVelocity = 10;
    ch.rods.push_back(r);
  }
  return c;
}
int main() {
  auto loaded = FFConfigIO::load("config/defaults/ff_config.json");
  assert(loaded);
  assert(loaded.value.channels[0].rods.size() == 14);
  assert(loaded.value.channels[1].rods.size() == 14);
  assert(loaded.value.channels[2].rods.size() == 12);
  assert(loaded.value.channels[0].trainingRod == RodId{12});
  assert(!loaded.value.channels[1].trainingRod);
  assert(!loaded.value.channels[2].trainingRod);
  for (const auto &channel : loaded.value.channels)
    for (const auto &rod : channel.rods) {
      assert(rod.maxVelocity == 1023);
      assert(rod.maxAcceleration == 15);
      assert(rod.maxDeceleration == 120);
      assert(rod.positionTolerance == 2);
      assert(rod.stableTime == std::chrono::milliseconds(100));
      assert(rod.moveTimeout == std::chrono::milliseconds(5000));
    }
  const auto sharedMotionPath =
      std::filesystem::temp_directory_path() / "ff_shared_motion.json";
  assert(FFConfigIO::save(sharedMotionPath.string(), loaded.value));
  const auto sharedMotionText = [&] {
    std::ifstream file(sharedMotionPath);
    return std::string(std::istreambuf_iterator<char>(file), {});
  }();
  assert(sharedMotionText.find("\"motor_parameters\"") != std::string::npos);
  assert(sharedMotionText.find("\"speed_limit_raw\"") ==
         sharedMotionText.rfind("\"speed_limit_raw\""));
  assert(FFConfigIO::load(sharedMotionPath.string()));
  std::filesystem::remove(sharedMotionPath);
  auto mixedMotion = loaded.value;
  mixedMotion.channels[0].rods[0].maxVelocity = 28;
  const auto mixedMotionPath =
      std::filesystem::temp_directory_path() / "ff_mixed_motion.json";
  assert(FFConfigIO::save(mixedMotionPath.string(), mixedMotion));
  auto reloadedMixedMotion = FFConfigIO::load(mixedMotionPath.string());
  assert(reloadedMixedMotion);
  assert(reloadedMixedMotion.value.channels[0].rods[0].maxVelocity == 28);
  assert(reloadedMixedMotion.value.channels[0].rods[1].maxVelocity == 1023);
  std::filesystem::remove(mixedMotionPath);
  auto invalidMotor = loaded.value;
  invalidMotor.channels[0].rods[0].maxAcceleration = 256;
  assert(!FFConfigIO::validate(invalidMotor));
  invalidMotor = loaded.value;
  invalidMotor.channels[0].rods[0].hardMax = 4096;
  assert(!FFConfigIO::validate(invalidMotor));
  auto c = cfg();
  {
    PositionFileData positions{"test","v1","three \"channels\"",{
        {{ChannelId::CH1,0},20},{{ChannelId::CH1,1},30},
        {{ChannelId::CH2,0},40},{{ChannelId::CH3,0},50}}};
    const auto path=(std::filesystem::temp_directory_path()/"ff_position_format.json").string();
    assert(PositionFiles::save(path,positions));
    std::ifstream input(path);
    const std::string written((std::istreambuf_iterator<char>(input)),{});
    assert(written.find("\"schema\": \"FF-POS-1\"")!=std::string::npos);
    assert(written.find("\"samples\"")==std::string::npos);
    assert(written.find("\"velocity\"")==std::string::npos);
    assert(written.find("{\"rod\": 0, \"position\": 20},\n"
                        "      {\"rod\": 1, \"position\": 30}")!=std::string::npos);
    for (int ch=0;ch<3;++ch)
      assert(written.find("\"channel\": "+std::to_string(ch))!=std::string::npos);
    auto restored=PositionFiles::load(path,c);
    assert(restored && restored.value.targets.size()==4 &&
           restored.value.summary==positions.summary);
    input.close();
    std::filesystem::remove(path);
  }
  {
    std::array<std::unique_ptr<IControllerBackend>,3> partial;
    partial[0]=std::make_unique<FailingOpenBackend>(false);
    partial[1]=std::make_unique<FailingOpenBackend>(true);
    partial[2]=std::make_unique<FailingOpenBackend>(false);
    FF failedOpen(c,std::move(partial));
    assert(failedOpen.initialize());
    assert(!failedOpen.connect());
    assert(!failedOpen.channel(ChannelId::CH1)->controller().isConnected());
    assert(!failedOpen.channel(ChannelId::CH3)->controller().isConnected());
  }
  assert(FFConfigIO::validate(c));
  auto invalidConfig = c;
  invalidConfig.channels[0].rods[0].softMin = -1;
  assert(!FFConfigIO::validate(invalidConfig));
  invalidConfig = c;
  invalidConfig.channels[0].rods.push_back(invalidConfig.channels[0].rods[0]);
  assert(!FFConfigIO::validate(invalidConfig));
  invalidConfig = c;
  invalidConfig.channels[0].trainingRod = RodId{99};
  assert(!FFConfigIO::validate(invalidConfig));

  // Stored controller ports are editable defaults.  The GUI may save blank or
  // duplicate values for disabled channels; ApplicationService replaces them
  // with the selected runtime ports before FF initialization.
  auto editablePorts = c;
  editablePorts.channels[0].controller.serial.portName.clear();
  editablePorts.channels[1].controller.serial.portName = "COM2";
  editablePorts.channels[2].controller.serial.portName = "COM2";
  assert(FFConfigIO::validate(editablePorts));
  const auto editablePath =
      std::filesystem::temp_directory_path() / "ff_editable_ports.json";
  assert(FFConfigIO::save(editablePath.string(), editablePorts));
  assert(FFConfigIO::load(editablePath.string()));
  std::filesystem::remove(editablePath);
  FFApplicationService editableService;
  assert(editableService.applyConfig(editablePorts, true, {"", "", ""}, 1));
  assert(editableService.applyConfig(editablePorts, false,
                                     {"COM7", "COM7", "COM7"}, 1));
  FFApplicationService duplicateRuntimePorts;
  assert(!duplicateRuntimePorts.applyConfig(
      editablePorts, false, {"COM7", "COM7", ""}, 2));
  FFApplicationService missingRuntimePort;
  assert(!missingRuntimePort.applyConfig(editablePorts, false,
                                         {"", "COM8", ""}, 2));
  std::array<std::unique_ptr<IControllerBackend>, 3> b;
  for (auto &i : b)
    i = makeSimulationBackend();
  FF f(c, std::move(b));
  assert(f.initialize());
  assert(f.connect());
  assert(f.refreshStatus());
  auto s = f.status();
  assert(s.channels[0].connected);

  // A loaded session may be retrained after the Training Rod has returned to
  // its initialized position.  Assignment must evaluate the Rod at its final
  // evasion position, not reject all candidate Rods against that temporary
  // initialized position.
  // Match the GUI loading path: Simulation supplies unique runtime ports even
  // if editable JSON defaults are intentionally blank or duplicated.
  FFApplicationService defaultService;
  assert(defaultService.applyConfig(loaded.value, true, {"", "", ""}, 3));
  assert(defaultService.connect());
  assert(defaultService.refresh());
  const auto &defaultChannel = loaded.value.channels[0];
  auto defaultStatus = defaultService.status();
  const auto trainingRodIt = std::find_if(
      defaultChannel.rods.begin(), defaultChannel.rods.end(),
      [](const RodConfig &rod) { return rod.id == RodId{12}; });
  assert(trainingRodIt != defaultChannel.rods.end());
  const auto trainingRodIndex = static_cast<size_t>(
      std::distance(defaultChannel.rods.begin(), trainingRodIt));
  // Keep the Training Rod at its initialized legal position.  Assignment may
  // move it only when the complete Layer 2 layout requires an evasion move.
  const auto loadedSessionAssignment = TrainingAnalysis::assign(
      loaded.value, defaultStatus, ChannelId::CH1, {17.3, 35.0, 52.5},
      RodId{12});
  assert(loadedSessionAssignment);
  std::vector<RodTarget> loadedSessionTargets;
  const auto liveDefaultStatus = defaultService.status();
  for (size_t index = 0; index < defaultChannel.rods.size(); ++index)
    loadedSessionTargets.push_back(
        {{ChannelId::CH1, defaultChannel.rods[index].id},
         liveDefaultStatus.channels[0].rods[index].actualPosition});
  for (auto &target : loadedSessionTargets) {
    for (const auto &aim : loadedSessionAssignment.value.aims)
      if (aim.assigned && target.address == aim.address)
        target.position = aim.encoderAim;
  }
  assert(defaultService.validateRodTargets(loadedSessionTargets));
  MotionRequest req{{{{ChannelId::CH1, 0}, 50}}, false};
  auto p = MotionPlanner::plan(c, s, req);
  assert(p);
  assert(p.value.steps.size() == 1);
  assert(p.value.steps[0].position == 50);
  auto layered = cfg();
  auto second = layered.channels[0].rods.front();
  second.id = 1;
  layered.channels[0].rods.push_back(second);
  layered.channels[0].rods[0].safePosition = 50;
  layered.channels[0].layerMembers = {
      {0, 4, 1, -1, 1}, {1, 4, 2, 0, -1}, {0, 9, 1, -1, 1}, {1, 9, 2, 0, -1}};
  layered.channels[0].clearanceRules.push_back({4, 0, 1, 5.0});
  layered.channels[0].clearanceRules.push_back({9, 0, 1, 5.0});
  auto layeredStatus = s;
  layeredStatus.safety[0].state = SafetyState::Safe;
  layeredStatus.channels[0].rods.push_back(layeredStatus.channels[0].rods[0]);
  layeredStatus.channels[0].rods[0].actualPosition = 40;
  layeredStatus.channels[0].rods[1].actualPosition = 80;
  auto layeredPlan = MotionPlanner::plan(
      layered, layeredStatus, {{{{ChannelId::CH1, 0}, 50}}, false});
  assert(layeredPlan);
  auto unsafeLayeredPlan = MotionPlanner::plan(
      layered, layeredStatus, {{{{ChannelId::CH1, 0}, 77}}, false});
  assert(!unsafeLayeredPlan);
  auto groupedPlan = MotionPlanner::plan(
      layered, layeredStatus,
      {{{{ChannelId::CH1, 0}, 50}, {{ChannelId::CH1, 1}, 85}}, false});
  assert(groupedPlan && groupedPlan.value.steps.size() == 2);
  static_assert(FF_MOTION_SPEED == 1023);
  auto groupedConfig = cfg();
  auto groupedRod = groupedConfig.channels[0].rods[0];
  groupedRod.id = 1;
  groupedConfig.channels[0].rods.push_back(groupedRod);
  groupedConfig.maxActiveRodCount = 4;
  FFApplicationService groupedService;
  assert(groupedService.applyConfig(groupedConfig, true, {"", "", ""}));
  assert(groupedService.connect());
  auto serviceGroupPlan = groupedService.previewPlan(
      {{{{ChannelId::CH1, 0}, 20},
         {{ChannelId::CH1, 1}, 80}}, false});
  assert(serviceGroupPlan && serviceGroupPlan.value.steps.size() == 2);
  assert(groupedService.executePlan(serviceGroupPlan.value));
  const auto groupedResult = groupedService.status();
  assert(groupedResult.channels[0].rods[0].actualPosition == 20);
  assert(groupedResult.channels[0].rods[1].actualPosition == 80);
  assert(groupedService.disconnect());
  auto noSafePosition = layered;
  for (auto &rod : noSafePosition.channels[0].rods)
    rod.safePositionValid = false;
  assert(MotionPlanner::plan(noSafePosition, layeredStatus,
                             {{{{ChannelId::CH1, 0}, 50},
                                {{ChannelId::CH1, 1}, 85}}, false}));

  // Clearance comparisons accept 0.10 mm of position/feedback variation,
  // but reject any larger shortfall.
  auto precisionLayered = layered;
  for (auto &rod : precisionLayered.channels[0].rods)
    rod.p1 = 0.01;
  precisionLayered.channels[0].rods[0].safePosition = 40;
  for (auto &rule : precisionLayered.channels[0].clearanceRules)
    rule.minimumClearanceMm = 0.50;
  auto precisionStatus = layeredStatus;
  precisionStatus.channels[0].rods[0].actualPosition = 40;
  precisionStatus.channels[0].rods[1].actualPosition = 90;
  auto toleranceAllowed = MotionPlanner::plan(
      precisionLayered, precisionStatus, {{{{ChannelId::CH1, 0}, 50}}, false});
  assert(toleranceAllowed);
  auto toleranceRejected = MotionPlanner::plan(
      precisionLayered, precisionStatus, {{{{ChannelId::CH1, 0}, 51}}, false});
  assert(!toleranceRejected);

  auto negativeClearance = layered;
  for (auto &rule : negativeClearance.channels[0].clearanceRules)
    rule.minimumClearanceMm = -5.0;
  auto allowedNegativeClearance = MotionPlanner::plan(
      negativeClearance, layeredStatus, {{{{ChannelId::CH1, 0}, 85}}, false});
  assert(allowedNegativeClearance);
  auto blockedNegativeClearance = MotionPlanner::plan(
      negativeClearance, layeredStatus, {{{{ChannelId::CH1, 0}, 86}}, false});
  assert(!blockedNegativeClearance);

  std::array<std::unique_ptr<IControllerBackend>, 3> layeredBackends;
  for (auto &backend : layeredBackends)
    backend = makeSimulationBackend();
  FF layeredMachine(layered, std::move(layeredBackends));
  assert(layeredMachine.initialize());
  assert(layeredMachine.connect());
  const auto simulatedInitialStatus = layeredMachine.status();
  assert(simulatedInitialStatus.safety[0].state == SafetyState::Safe);
  assert(simulatedInitialStatus.channels[0].rods[0].actualPosition !=
         simulatedInitialStatus.channels[0].rods[1].actualPosition);
  // Establish a simulated neighbor position, then verify that the common
  // submission path rejects a target that violates the 5 mm clearance rule.
  assert(layeredMachine.channel(ChannelId::CH1)
             ->controller()
             .submit({{ChannelId::CH1, 1}, 40}));
  const auto beforeRejectedTarget = layeredMachine.status();
  MotionRequest simulatedUnsafeRequest{{{{ChannelId::CH1, 0}, 43}},
                                       false};
  auto simulatedUnsafe = layeredMachine.submitMotion(simulatedUnsafeRequest);
  assert(!simulatedUnsafe);
  const auto afterRejectedTarget = layeredMachine.status();
  assert(afterRejectedTarget.channels[0].rods[0].actualPosition ==
         beforeRejectedTarget.channels[0].rods[0].actualPosition);
  assert(afterRejectedTarget.channels[0].rods[0].targetPosition ==
         beforeRejectedTarget.channels[0].rods[0].targetPosition);

  // A command checks the commanded Rod's own neighbors, but a transient
  // clearance reading from an unrelated stationary pair must not block it.
  auto scoped = cfg();
  for (RodId id = 1; id < 4; ++id) {
    auto rod = scoped.channels[0].rods.front();
    rod.id = id;
    scoped.channels[0].rods.push_back(rod);
  }
  scoped.channels[0].layerMembers = {
      {0, 4, 1, -1, 1}, {1, 4, 2, 0, -1},
      {2, 5, 1, -1, 3}, {3, 5, 2, 2, -1}};
  scoped.channels[0].clearanceRules = {{4, 0, 1, 5.0}, {5, 2, 3, 5.0}};
  std::array<std::unique_ptr<IControllerBackend>, 3> scopedBackends;
  for (auto &backend : scopedBackends)
    backend = makeSimulationBackend();
  FF scopedMachine(scoped, std::move(scopedBackends));
  assert(scopedMachine.initialize());
  assert(scopedMachine.connect());
  // Establish a valid pair for Rod 0, then inject an invalid, non-commanded
  // pair through the backend to model independent feedback vibration.
  assert(scopedMachine.channel(ChannelId::CH1)->controller().submit(
      {{ChannelId::CH1, 0}, 40}));
  assert(scopedMachine.channel(ChannelId::CH1)->controller().submit(
      {{ChannelId::CH1, 1}, 80}));
  assert(scopedMachine.channel(ChannelId::CH1)->controller().submit(
      {{ChannelId::CH1, 2}, 78}));
  assert(scopedMachine.channel(ChannelId::CH1)->controller().submit(
      {{ChannelId::CH1, 3}, 80}));
  MotionRequest scopedRequest{{{{ChannelId::CH1, 0}, 50}}, false};
  assert(scopedMachine.submitMotion(scopedRequest));

  auto probeConfig = cfg();
  auto probeNeighbor = probeConfig.channels[0].rods.front();
  probeNeighbor.id=1;
  probeConfig.channels[0].rods.push_back(probeNeighbor);
  probeConfig.channels[0].layerMembers={{0,2,1,-1,1},{1,2,2,0,-1}};
  probeConfig.channels[0].clearanceRules={{2,0,1,5.0}};
  auto probeState=std::make_shared<ProbeState>();
  std::array<std::unique_ptr<IControllerBackend>,3> probeBackends;
  probeBackends[0]=std::make_unique<ProbeBackend>(probeState);
  probeBackends[1]=makeSimulationBackend();
  probeBackends[2]=makeSimulationBackend();
  FF probeMachine(probeConfig,std::move(probeBackends));
  assert(probeMachine.initialize()); assert(probeMachine.connect());
  const MotionRequest probeSafe{{{{ChannelId::CH1,0},20}},false};
  const MotionRequest probeConflict{{{{ChannelId::CH1,0},89}},false};
  assert(!probeMachine.submitMotion(probeConflict));
  assert(probeState->commands==0);
  probeState->invalidFeedback=true;
  assert(!probeMachine.submitMotion(probeSafe));
  assert(probeState->commands==0);
  probeState->invalidFeedback=false; probeState->alarm=true;
  assert(!probeMachine.submitMotion(probeSafe));
  assert(probeState->commands==0);
  probeState->alarm=false;
  const MotionRequest probeGroup{{{{ChannelId::CH1,0},20},
                                  {{ChannelId::CH1,1},80}},false};
  assert(probeMachine.submitMotion(probeGroup));
  assert(probeState->commands==1);
  const MotionRequest probeUnsafeTarget{{{{ChannelId::CH1,0},89},
                                         {{ChannelId::CH1,1},70}},false};
  assert(!probeMachine.submitMotion(probeUnsafeTarget));
  assert(probeState->commands==1);

  MotionRequest badReq{{{{ChannelId::CH1, 0}, 101}}, false};
  auto bad = f.submitMotion(badReq);
  assert(!bad);
  assert(f.emergencyStop());
  FFApplicationService service;
  assert(service.applyConfig(c, true, {"", "", ""}));
  assert(service.connect());
  assert(!service.applyConfig(c, true, {"", "", ""}));
  assert(service.disconnect());
  auto slowConfig=cfg();
  slowConfig.channels[0].rods[0].stableTime=std::chrono::milliseconds(2000);
  slowConfig.channels[0].rods[0].moveTimeout=std::chrono::milliseconds(80);
  assert(service.applyConfig(slowConfig,true,{"","",""}));
  assert(service.connect());
  auto timed=service.moveRodAndWait({{ChannelId::CH1,0},60});
  assert(!timed && timed.error==ErrorCode::Timeout);
  assert(service.disconnect());
  slowConfig.channels[0].rods[0].moveTimeout=std::chrono::milliseconds(5000);
  assert(service.applyConfig(slowConfig,true,{"","",""}));
  assert(service.connect());
  Result<void> interrupted;
  std::thread moving([&]{ interrupted=service.moveRodAndWait({{ChannelId::CH1,0},65}); });
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  assert(service.cancelMotion());
  moving.join();
  assert(!interrupted && interrupted.error==ErrorCode::Busy);
  assert(service.disconnect());
  auto forbiddenConfig=cfg();
  forbiddenConfig.channels[0].rods[0].forbiddenRanges={{55,60}};
  assert(service.applyConfig(forbiddenConfig,true,{"","",""}));
  assert(service.connect());
  const auto beforeForbidden=service.status().channels[0].rods[0].actualPosition;
  auto forbiddenMove=service.moveRodAndWait({{ChannelId::CH1,0},65});
  assert(!forbiddenMove && forbiddenMove.error==ErrorCode::SafetyViolation);
  assert(service.status().channels[0].rods[0].actualPosition==beforeForbidden);
  assert(service.disconnect());
  auto noSafePositionConfig = c;
  for (auto &channel : noSafePositionConfig.channels)
    for (auto &rod : channel.rods)
      rod.safePositionValid = false;
  assert(service.applyConfig(noSafePositionConfig, true, {"", "", ""}));
  assert(service.connect());
  const auto noSafePositionStatus = service.status();
  assert(service.validateRodTargets(
      {{{ChannelId::CH1, 0},
        noSafePositionStatus.channels[0].rods[0].actualPosition}}));
  assert(service.disconnect());

  // Training analysis is independent of the GUI and therefore remains
  // executable in the core test binary.
  std::vector<TrainingSample> trainingSamples;
  for (int i = 0; i <= 200; ++i) {
    const double position = i * .05;
    trainingSamples.push_back(
        {position, TrainingAnalysis::simulatedEnergy(TrainingSource::Sim1,
                                                       position, 0, 10)});
  }
  ValleyDetectionSettings valleySettings;
  valleySettings.maximumCount = 3;
  const auto valleys = TrainingAnalysis::findValleys(trainingSamples, valleySettings);
  assert(valleys.size() == 3);
  assert(std::abs(valleys[1] - 5.0) < .35);
  assert(TrainingAnalysis::interpolateEnergy(trainingSamples, 5.0) <
         TrainingAnalysis::interpolateEnergy(trainingSamples, 4.0));

  auto assignmentConfig = cfg();
  auto secondRod = assignmentConfig.channels[0].rods[0];
  secondRod.id = 1;
  secondRod.p0 = 5.0;
  assignmentConfig.channels[0].rods.push_back(secondRod);
  assignmentConfig.channels[0].layerMembers = {
      {0, 2, 3, -1, -1}, {1, 2, 3, -1, -1}};
  auto assignmentStatus = s;
  assignmentStatus.channels[0].rods.push_back(assignmentStatus.channels[0].rods[0]);
  assignmentStatus.channels[0].rods[0].actualPosition = 20;
  assignmentStatus.channels[0].rods[1].actualPosition = 20;
  auto assignment = TrainingAnalysis::assign(assignmentConfig, assignmentStatus,
                                             ChannelId::CH1, {25.0, 60.0});
  assert(assignment);
  assert(assignment.value.aims.size() == 2);
  assert(TrainingAnalysis::validateAssignment(assignmentConfig, assignmentStatus,
                                              ChannelId::CH1, {25.0, 60.0}));
  auto excludedTrainingRod = TrainingAnalysis::assign(
      assignmentConfig, assignmentStatus, ChannelId::CH1, {25.0}, RodId{1});
  assert(excludedTrainingRod);
  assert(excludedTrainingRod.value.aims.front().address.rod == RodId{0});

  const std::vector<double> dragCandidates{10.0, 20.0, 30.0};
  const auto middleDrag = TrainingAnalysis::dragInterval(
      dragCandidates, 1, 20.0, 0.0, 40.0);
  assert(middleDrag.valid);
  assert(std::abs(middleDrag.minimumMm - 13.0) < 1e-9);
  assert(std::abs(middleDrag.maximumMm - 27.0) < 1e-9);
  const auto reverseDrag = TrainingAnalysis::dragInterval(
      dragCandidates, 1, 20.0, 40.0, 0.0);
  assert(reverseDrag.valid);
  assert(reverseDrag.minimumMm == middleDrag.minimumMm);
  assert(reverseDrag.maximumMm == middleDrag.maximumMm);
  const auto edgeDrag = TrainingAnalysis::dragInterval(
      {5.0}, 0, 5.0, 0.0, 40.0);
  assert(edgeDrag.valid);
  assert(std::abs(edgeDrag.minimumMm - 1.5) < 1e-9);
  assert(std::abs(edgeDrag.maximumMm - 38.5) < 1e-9);
  assert(!TrainingAnalysis::dragInterval(
              dragCandidates, dragCandidates.size(), 20.0, 0.0, 40.0)
              .valid);

  TrainingSession session;
  session.machineId = "test";
  session.configVersion = "v1";
  session.samples = trainingSamples;
  session.samples[0].energy=9007199254740993ULL;
  session.candidatesMm = valleys;
  session.aims = assignment.value.aims;
  session.occlusionSummary =
      "CH1; Training Rod R1; 2 obstruction positions; assigned R0";
  session.rodStates = {
      {{ChannelId::CH1, 0}, 20, 25},
      {{ChannelId::CH1, 1}, 20, 55}};
  session.executionTargets = {
      {{ChannelId::CH1, 0}, 25},
      {{ChannelId::CH1, 1}, 55}};
  const auto sessionPath =
      (std::filesystem::temp_directory_path() / "ff_training_test.json").string();
  assert(TrainingAnalysis::saveSession(sessionPath, session));
  std::ifstream writtenSession(sessionPath);
  const std::string sessionJson((std::istreambuf_iterator<char>(writtenSession)), {});
  writtenSession.close();
  assert(sessionJson.find("\"velocity\"") == std::string::npos);
  assert(sessionJson.find("0.10000000000000001") == std::string::npos);
  assert(sessionJson.find("\"samples\": [\n") != std::string::npos);
  auto legacySessionJson = sessionJson;
  const auto firstTargetPosition = legacySessionJson.find("\"position\": 25");
  assert(firstTargetPosition != std::string::npos);
  legacySessionJson.insert(firstTargetPosition + std::string("\"position\": 25").size(),
                           ",\"velocity\":10");
  const auto legacySessionPath = sessionPath + ".legacy";
  std::ofstream(legacySessionPath) << legacySessionJson;
  assert(TrainingAnalysis::loadSession(legacySessionPath));
  auto restoredSession = TrainingAnalysis::loadSession(sessionPath);
  assert(restoredSession);
  assert(restoredSession.value.samples.size() == session.samples.size());
  assert(restoredSession.value.samples[0].energy==9007199254740993ULL);
  assert(restoredSession.value.aims.size() == session.aims.size());
  assert(restoredSession.value.occlusionSummary == session.occlusionSummary);
  assert(restoredSession.value.rodStates.size() == 2);
  assert(restoredSession.value.rodStates[1].actualEncoder == 20);
  assert(restoredSession.value.rodStates[1].targetEncoder == 55);
  assert(restoredSession.value.executionTargets.size() == 2);
  assert(restoredSession.value.executionTargets[1].address.rod == RodId{1});
  assert(restoredSession.value.executionTargets[1].position == 55);
  TrainingArchive archive;
  archive.machineId="test"; archive.configVersion="v1";
  archive.createdUnixSeconds=1794832729;
  archive.selectedChannel=0;
  session.samples[0].positionMm=23.401245;
  archive.channels[0]=session;
  auto secondChannel=session;
  secondChannel.channel=ChannelId::CH2;
  for (auto& aim:secondChannel.aims) aim.address.channel=ChannelId::CH2;
  for (auto& state:secondChannel.rodStates) state.address.channel=ChannelId::CH2;
  for (auto& target:secondChannel.executionTargets) target.address.channel=ChannelId::CH2;
  archive.channels[1]=secondChannel;
  const auto archivePath=(std::filesystem::temp_directory_path()/"ff_training_archive.json").string();
  assert(TrainingAnalysis::saveArchive(archivePath,archive));
  std::ifstream writtenArchive(archivePath);
  const std::string archiveJson((std::istreambuf_iterator<char>(writtenArchive)),{});
  writtenArchive.close();
  assert(archiveJson.find("\"schema\": \"FF-TRAIN-2\"")!=std::string::npos);
  assert(archiveJson.find("\"position_mm\": 23.401")!=std::string::npos);
  assert(archiveJson.find("23.401245")==std::string::npos);
  assert(archiveJson.find("0.10000000000000001")==std::string::npos);
  assert(archiveJson.find("\"channel\": 2, \"training\": {}")!=std::string::npos);
  auto restoredArchive=TrainingAnalysis::loadArchive(archivePath);
  assert(restoredArchive && restoredArchive.value.channels[0] &&
         restoredArchive.value.channels[1] && !restoredArchive.value.channels[2]);
  assert(restoredArchive.value.channels[0]->samples[0].positionMm==23.401);
  assert(restoredArchive.value.channels[1]->executionTargets[1].position==55);
  auto archivePositions=PositionFiles::load(archivePath,c);
  assert(archivePositions && archivePositions.value.targets.size()==4);
  auto oldArchive=TrainingAnalysis::loadArchive(legacySessionPath);
  assert(oldArchive && oldArchive.value.channels[0] &&
         !oldArchive.value.channels[1] && !oldArchive.value.channels[2]);
  auto tooSmallStep=session;
  tooSmallStep.stepMm=0.0001;
  assert(!TrainingAnalysis::saveSession(archivePath+".invalid",tooSmallStep));
  std::filesystem::remove(archivePath);
  std::filesystem::remove(sessionPath);
  std::filesystem::remove(legacySessionPath);
  std::cout << "FF tests passed\n";
}
