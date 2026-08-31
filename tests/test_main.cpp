#include "ff/FF.hpp"
#include <cassert>
#include <iostream>
using namespace ff;
static FFConfig cfg() {
  FFConfig c;
  c.machineId = "test";
  c.configVersion = "v1";
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
  auto c = cfg();
  std::array<std::unique_ptr<IControllerBackend>, 3> b;
  for (auto &i : b)
    i = makeSimulationBackend();
  FF f(c, std::move(b));
  assert(f.initialize());
  assert(f.connect());
  assert(f.refreshStatus());
  auto s = f.status();
  assert(s.channels[0].connected);
  MotionRequest req{{{{ChannelId::CH1, 0}, 50, 5}}, false};
  auto p = MotionPlanner::plan(c, s, req);
  assert(p);
  assert(p.value.steps.size() == 2);
  auto layered = cfg();
  auto second = layered.channels[0].rods.front();
  second.id = 1;
  layered.channels[0].rods.push_back(second);
  layered.channels[0].layerMembers = {
      {0, 4, 1, -1, 1}, {1, 4, 2, 0, -1}, {0, 9, 1, -1, 1}, {1, 9, 2, 0, -1}};
  layered.channels[0].clearanceRules.push_back({4, 0, 1, 5.0});
  layered.channels[0].clearanceRules.push_back({9, 0, 1, 5.0});
  auto layeredStatus = s;
  layeredStatus.safety[0].state = SafetyState::Safe;
  layeredStatus.channels[0].rods.push_back(layeredStatus.channels[0].rods[0]);
  layeredStatus.channels[0].rods[0].actualPosition = 80;
  layeredStatus.channels[0].rods[1].actualPosition = 40;
  auto layeredPlan = MotionPlanner::plan(
      layered, layeredStatus, {{{{ChannelId::CH1, 0}, 50, 5}}, false});
  assert(layeredPlan);
  auto unsafeLayeredPlan = MotionPlanner::plan(
      layered, layeredStatus, {{{{ChannelId::CH1, 0}, 43, 5}}, false});
  assert(!unsafeLayeredPlan);
  MotionRequest badReq{{{{ChannelId::CH1, 0}, 101, 1}}, false};
  auto bad = f.submitMotion(badReq);
  assert(!bad);
  assert(f.emergencyStop());
  std::cout << "FF tests passed\n";
}
