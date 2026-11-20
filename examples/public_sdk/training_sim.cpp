// Simulation-only workflow. No stage or camera SDK is invoked.
#include <DX_FF.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
bool same(FF::Address a, FF::Address b) {
  return a.channel == b.channel && a.rod == b.rod;
}
void check(const FF::Result& r) {
  if (!r) throw std::runtime_error("FF code=" + std::to_string(int(r.code)) + ": " + r.message);
}
FF::RodPosition feedback(const FF::ff_driver& driver, FF::Address address) {
  const auto& all = driver.m_current_positions;
  auto p = std::find_if(all.begin(), all.end(), [&](const auto& v) { return same(v.address, address); });
  if (p == all.end() || !p->valid) throw std::runtime_error("Missing/invalid position feedback");
  const auto& states = driver.m_status.channels[address.channel].rods;
  auto s = std::find_if(states.begin(), states.end(), [&](const auto& v) { return same(v.address, address); });
  if (s == states.end() || !s->positionValid || s->alarm)
    throw std::runtime_error("Missing/alarmed Rod status");
  return *p; // Copy; later SDK calls may update the snapshots.
}
struct Scan {
  FF::RodInfo rod;
  double start{}, end{};
};
FF::Target target(const Scan& scan, int i, int count) {
  const double mm = scan.start + (scan.end - scan.start) * i / (count - 1);
  const double raw = (mm - scan.rod.p0) / scan.rod.p1;
  if (!std::isfinite(raw) || raw < std::numeric_limits<std::int32_t>::min() ||
      raw > std::numeric_limits<std::int32_t>::max())
    throw std::runtime_error("Unrepresentable scan target");
  const auto encoder = static_cast<std::int32_t>(std::llround(raw));
  if (encoder < scan.rod.softMin || encoder > scan.rod.softMax ||
      encoder < scan.rod.hardMin || encoder > scan.rod.hardMax)
    throw std::runtime_error("Scan target outside limits");
  return {scan.rod.address, encoder};
}
void wait_round(FF::ff_driver& driver, const std::vector<FF::Target>& targets,
                const std::array<Scan, 3>& scans) {
  // These timing values belong to the accompanying simulation fixture.
  // A hardware adapter must use the commissioned per-device timeout/stability settings.
  const auto began = Clock::now();
  std::array<std::optional<Clock::time_point>, 3> stable{};
  for (;;) {
    check(driver.refresh());
    if (!driver.m_status.initialized) throw std::runtime_error("Initialization was invalidated");
    const auto now = Clock::now();
    bool ready = true;
    for (const auto& t : targets) {
      const int ch = t.address.channel;
      const auto p = feedback(driver, t.address);
      if (std::abs(double(p.encoder) - t.position) > scans[ch].rod.positionTolerance)
        stable[ch].reset();
      else if (!stable[ch]) stable[ch] = now;
      if (!stable[ch] || now - *stable[ch] < std::chrono::milliseconds(20)) ready = false;
    }
    if (ready) return;
    if (now - began > std::chrono::milliseconds(1000)) throw std::runtime_error("Scan point timed out");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
}

int main(int argc, char** argv) {
  if (argc < 4 || argc > 7) {
    std::cerr << "usage: training_sim <training_config.sim.json> <training_slot_0.json>"
                 " <new-output-directory> [channel-mask:1..7] [forward|reverse] [valleys|flat]\n";
    return 2;
  }
  FF::ff_driver driver;
  FF::TrainingArchive archive;
  std::filesystem::path output;
  bool opened = false, archiveReady = false;
  try {
    const std::string maskText = argc > 4 ? argv[4] : "7";
    if (maskText.size() != 1 || maskText[0] < '1' || maskText[0] > '7')
      throw std::runtime_error("Channel mask must be 1..7");
    const int mask = maskText[0] - '0';
    const std::string direction = argc > 5 ? argv[5] : "forward";
    const std::string profile = argc > 6 ? argv[6] : "valleys";
    if ((direction != "forward" && direction != "reverse") ||
        (profile != "valleys" && profile != "flat"))
      throw std::runtime_error("Invalid direction/profile");
    output = argv[3];
    // Never overwrite a previous successful run.
    if (!std::filesystem::create_directory(output))
      throw std::runtime_error("Output directory must not already exist (parent must exist)");
    FF::ChannelSelection enabled{bool(mask & 1), bool(mask & 2), bool(mask & 4)};
    check(driver.open(argv[1], FF::Mode::Simulation, {}, enabled));
    opened = true;
    check(driver.initialize(argv[2]));
    if (!driver.m_status.initialized) throw std::runtime_error("Initialization incomplete");
    const auto info = driver.machine_info();
    constexpr int count = 300;
    archive.machineId = driver.m_config.machineId;
    archive.configVersion = driver.m_config.configVersion;
    archive.enabledChannels = enabled;
    archive.sampleCount = count;
    archive.createdUnixSeconds = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::array<Scan, 3> scans{};
    bool selected = false;
    for (int ch = 0; ch < 3; ++ch) {
      if (!enabled[ch]) continue;
      const auto& channel = info.channels[ch];
      auto rod = std::find_if(channel.rods.begin(), channel.rods.end(),
          [&](const auto& r) { return r.address.rod == channel.trainingRod; });
      if (channel.trainingRod < 0 || rod == channel.rods.end() ||
          !std::isfinite(rod->p0) || !std::isfinite(rod->p1) || std::abs(rod->p1) < 1e-12)
        throw std::runtime_error("Configured Training Rod/calibration is missing");
      scans[ch] = {*rod, rod->p0 + rod->p1 * rod->softMin, rod->p0 + rod->p1 * rod->softMax};
      if (direction == "reverse") std::swap(scans[ch].start, scans[ch].end);
      FF::TrainingSession session;
      session.channel = ch;
      session.trainingRod = channel.trainingRod;
      session.source = FF::TrainingSource::Sim1;
      session.startMm = scans[ch].start;
      session.endMm = scans[ch].end;
      session.sampleCount = count;
      session.valleySettings.maximumCount = 1; // Fixture has one non-scanning candidate Rod.
      session.machineId = archive.machineId;
      session.configVersion = archive.configVersion;
      session.createdUnixSeconds = archive.createdUnixSeconds;
      session.occlusionSummary = "Simulation only; profile=" + profile;
      archive.channels[ch] = session;
      if (!selected) { archive.selectedChannel = ch; selected = true; }
    }
    archiveReady = true;
    check(driver.refresh());
    for (int i = 0; i < count; ++i) {
      std::vector<FF::Target> round;
      for (int ch = 0; ch < 3; ++ch) if (enabled[ch]) round.push_back(target(scans[ch], i, count));
      check(driver.validate_rods(round)); // No motion in preflight.
    }
    for (int i = 0; i < count; ++i) {
      std::vector<FF::Target> round;
      for (int ch = 0; ch < 3; ++ch) if (enabled[ch]) round.push_back(target(scans[ch], i, count));
      check(driver.send_partial_rods(round));
      wait_round(driver, round, scans);
      std::array<FF::Sample, 3> samples{};
      for (int ch = 0; ch < 3; ++ch) {
        if (!enabled[ch]) continue;
        const auto p = feedback(driver, scans[ch].rod.address);
        const auto energy = profile == "flat" ? std::uint64_t{1000000} :
            driver.simulated_energy(FF::TrainingSource::Sim1, p.opticalMm, scans[ch].start, scans[ch].end);
        samples[ch] = {p.opticalMm, energy};
      }
      // Commit only after every enabled channel has a sample.
      for (int ch = 0; ch < 3; ++ch) if (enabled[ch]) archive.channels[ch]->samples.push_back(samples[ch]);
    }
    for (int ch = 0; ch < 3; ++ch) if (enabled[ch]) archive.channels[ch]->complete = true;
    check(driver.refresh());
    std::vector<FF::Target> execution;
    for (const auto& p : driver.m_current_positions) {
      feedback(driver, p.address);
      execution.push_back({p.address, p.encoder});
    }
    for (int ch = 0; ch < 3; ++ch) {
      if (!enabled[ch]) continue;
      auto& session = *archive.channels[ch];
      session.candidatesMm = driver.find_valleys(session.samples, session.valleySettings);
      auto assignment = driver.assign_training(ch, session.candidatesMm, session.trainingRod);
      check(assignment.result);
      session.aims = assignment.value.aims;
      for (const auto& aim : session.aims) {
        auto t = std::find_if(execution.begin(), execution.end(),
            [&](const auto& v) { return same(v.address, aim.address); });
        if (t == execution.end()) throw std::runtime_error("Assignment address missing");
        t->position = aim.encoderAim; // Include evasion aims (assigned == false).
      }
      std::cout << "CH" << ch + 1 << ": " << session.samples.size()
                << " samples, " << session.candidatesMm.size() << " candidates\n";
    }
    check(driver.refresh());
    check(driver.validate_rods(execution));
    check(driver.move_rods(execution));
    check(driver.refresh());
    for (int ch = 0; ch < 3; ++ch) {
      if (!enabled[ch]) continue;
      auto& session = *archive.channels[ch];
      for (const auto& t : execution) if (t.address.channel == ch) {
        const auto p = feedback(driver, t.address);
        session.executionTargets.push_back(t);
        session.rodStates.push_back({t.address, p.encoder, t.position});
      }
    }
    const auto sessionPath = (output / "training.json").string();
    const auto positionPath = (output / "positions.json").string();
    check(driver.save_training_archive(sessionPath, archive));
    check(driver.record_positions(positionPath, "simulation result; no optical acceptance"));
    auto reloaded = driver.load_training_archive(sessionPath);
    check(reloaded.result);
    if (reloaded.value.enabledChannels != enabled || reloaded.value.sampleCount != count)
      throw std::runtime_error("Archive readback metadata mismatch");
    for (int ch = 0; ch < 3; ++ch) if (enabled[ch]) {
      if (!reloaded.value.channels[ch] || !reloaded.value.channels[ch]->complete ||
          reloaded.value.channels[ch]->samples.size() != count)
        throw std::runtime_error("Archive readback sample mismatch");
    }
    auto positions = driver.load_positions(positionPath);
    check(positions.result);
    check(driver.move_rods(positions.value.targets)); // Replay the separately saved position recipe.
    check(driver.close());
    opened = false;
    std::cout << "Simulation workflow passed. Output: " << output.string() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    if (opened) {
      const auto stopped = driver.stop();
      if (!stopped) std::cerr << "Cleanup stop: " << stopped.message << '\n';
      if (archiveReady) {
        const auto saved = driver.save_training_archive((output / "partial_training.json").string(), archive);
        if (!saved) std::cerr << "Partial archive: " << saved.message << '\n';
      }
      const auto closed = driver.close();
      if (!closed) std::cerr << "Cleanup close: " << closed.message << '\n';
    }
    return 1;
  }
}
