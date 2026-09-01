#include "ff/FF.hpp"
#include "ff/Training.hpp"
#include <cassert>
#include <filesystem>
#include <iostream>
using namespace ff;
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
  auto c = cfg();
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
  std::array<std::unique_ptr<IControllerBackend>, 3> defaultBackends;
  for (auto &backend : defaultBackends)
    backend = makeSimulationBackend();
  FF defaultMachine(loaded.value, std::move(defaultBackends));
  assert(defaultMachine.initialize());
  assert(defaultMachine.connect());
  assert(defaultMachine.refreshStatus());
  const auto &defaultChannel = loaded.value.channels[0];
  auto defaultStatus = defaultMachine.status();
  const auto trainingRodIt = std::find_if(
      defaultChannel.rods.begin(), defaultChannel.rods.end(),
      [](const RodConfig &rod) { return rod.id == RodId{12}; });
  assert(trainingRodIt != defaultChannel.rods.end());
  const auto trainingRodIndex = static_cast<size_t>(
      std::distance(defaultChannel.rods.begin(), trainingRodIt));
  defaultStatus.channels[0].rods[trainingRodIndex].actualPosition =
      trainingRodIt->softMax;
  defaultStatus.channels[0].rods[trainingRodIndex].targetPosition =
      trainingRodIt->softMax;
  const auto loadedSessionAssignment = TrainingAnalysis::assign(
      loaded.value, defaultStatus, ChannelId::CH1, {17.3, 35.0, 52.5},
      RodId{12});
  assert(loadedSessionAssignment);
  std::vector<RodTarget> loadedSessionTargets;
  const auto liveDefaultStatus = defaultMachine.status();
  for (size_t index = 0; index < defaultChannel.rods.size(); ++index)
    loadedSessionTargets.push_back(
        {{ChannelId::CH1, defaultChannel.rods[index].id},
         liveDefaultStatus.channels[0].rods[index].actualPosition,
         defaultChannel.rods[index].maxVelocity});
  for (auto &target : loadedSessionTargets) {
    if (target.address.rod == RodId{12})
      target.position = trainingRodIt->softMax;
    for (const auto &aim : loadedSessionAssignment.value.aims)
      if (aim.assigned && target.address == aim.address)
        target.position = aim.encoderAim;
  }
  assert(defaultMachine.validateMotion({loadedSessionTargets, false}));
  MotionRequest req{{{{ChannelId::CH1, 0}, 50, 5}}, false};
  auto p = MotionPlanner::plan(c, s, req);
  assert(p);
  assert(p.value.steps.size() == 2);
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
      layered, layeredStatus, {{{{ChannelId::CH1, 0}, 50, 5}}, false});
  assert(layeredPlan);
  auto unsafeLayeredPlan = MotionPlanner::plan(
      layered, layeredStatus, {{{{ChannelId::CH1, 0}, 77, 5}}, false});
  assert(!unsafeLayeredPlan);

  auto negativeClearance = layered;
  for (auto &rule : negativeClearance.channels[0].clearanceRules)
    rule.minimumClearanceMm = -5.0;
  auto allowedNegativeClearance = MotionPlanner::plan(
      negativeClearance, layeredStatus, {{{{ChannelId::CH1, 0}, 85, 5}}, false});
  assert(allowedNegativeClearance);
  auto blockedNegativeClearance = MotionPlanner::plan(
      negativeClearance, layeredStatus, {{{{ChannelId::CH1, 0}, 86, 5}}, false});
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
             .submit({{ChannelId::CH1, 1}, 40, 5}));
  const auto beforeRejectedTarget = layeredMachine.status();
  MotionRequest simulatedUnsafeRequest{{{{ChannelId::CH1, 0}, 43, 5}},
                                       false};
  auto simulatedUnsafe = layeredMachine.submitMotion(simulatedUnsafeRequest);
  assert(!simulatedUnsafe);
  const auto afterRejectedTarget = layeredMachine.status();
  assert(afterRejectedTarget.channels[0].rods[0].actualPosition ==
         beforeRejectedTarget.channels[0].rods[0].actualPosition);
  assert(afterRejectedTarget.channels[0].rods[0].targetPosition ==
         beforeRejectedTarget.channels[0].rods[0].targetPosition);

  MotionRequest badReq{{{{ChannelId::CH1, 0}, 101, 1}}, false};
  auto bad = f.submitMotion(badReq);
  assert(!bad);
  assert(f.emergencyStop());
  FFApplicationService service;
  assert(service.applyConfig(c, true, {"", "", ""}));
  assert(service.connect());
  assert(!service.applyConfig(c, true, {"", "", ""}));
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
        noSafePositionStatus.channels[0].rods[0].actualPosition, 10.0}}));
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
  const std::uint8_t gray[] = {0, 1, 127, 255};
  assert(TrainingAnalysis::accumulateGray(gray, 4) == 383);
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
  session.candidatesMm = valleys;
  session.aims = assignment.value.aims;
  session.occlusionSummary =
      "CH1; Training Rod R1; 2 obstruction positions; assigned R0";
  session.rodStates = {
      {{ChannelId::CH1, 0}, 20, 25},
      {{ChannelId::CH1, 1}, 20, 55}};
  session.executionTargets = {
      {{ChannelId::CH1, 0}, 25, 10.0},
      {{ChannelId::CH1, 1}, 55, 10.0}};
  const auto sessionPath =
      (std::filesystem::temp_directory_path() / "ff_training_test.json").string();
  assert(TrainingAnalysis::saveSession(sessionPath, session));
  auto restoredSession = TrainingAnalysis::loadSession(sessionPath);
  assert(restoredSession);
  assert(restoredSession.value.samples.size() == session.samples.size());
  assert(restoredSession.value.aims.size() == session.aims.size());
  assert(restoredSession.value.occlusionSummary == session.occlusionSummary);
  assert(restoredSession.value.rodStates.size() == 2);
  assert(restoredSession.value.rodStates[1].actualEncoder == 20);
  assert(restoredSession.value.rodStates[1].targetEncoder == 55);
  assert(restoredSession.value.executionTargets.size() == 2);
  assert(restoredSession.value.executionTargets[1].address.rod == RodId{1});
  assert(restoredSession.value.executionTargets[1].position == 55);
  std::filesystem::remove(sessionPath);
  std::cout << "FF tests passed\n";
}
