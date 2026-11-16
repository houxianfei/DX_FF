#include <DX_FF.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: example <config.json> <slot_0.json> <record.json>\n";
    return 2;
  }
  FF::ff_driver driver;
  auto opened = driver.open(argv[1], FF::Mode::Simulation);
  if (!opened) { std::cerr << opened.message << '\n'; return 1; }
  auto initial = driver.load_positions(argv[2]);
  if (!initial) { std::cerr << initial.result.message << '\n'; return 1; }
  auto moved = driver.move_rods(initial.value.targets);
  if (!moved) { std::cerr << moved.message << '\n'; return 1; }
  auto recorded = driver.record_positions(argv[3], "SDK example");
  if (!recorded) { std::cerr << recorded.message << '\n'; return 1; }
  auto loaded = driver.load_positions(argv[3]);
  if (!loaded) { std::cerr << loaded.result.message << '\n'; return 1; }
  auto executed = driver.move_rods(loaded.value.targets);
  if (!executed) { std::cerr << executed.message << '\n'; return 1; }
  if (!driver.m_current_positions.empty()) {
    const auto& rod=driver.m_current_positions.front();
    auto moved=driver.move_rod(rod.address,rod.encoder);
    if (!moved) { std::cerr << moved.message << '\n'; return 1; }
  }
  std::vector<FF::Sample> samples{{0.0,100},{1.0,30},{2.0,100}};
  auto valleys = driver.find_valleys(samples,{});
  std::cout << "SDK " << driver.version() << ", state " << driver.m_status.state
            << ", valleys " << valleys.size() << '\n';
  driver.close();
}
