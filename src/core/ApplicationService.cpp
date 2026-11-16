#include "ff/FF.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>

namespace ff {
struct FFApplicationService::Impl {
  mutable std::mutex mutex;
  std::unique_ptr<FF> machine;
  bool simulation{true};
  int activeChannelCount{3};
  bool connected{false};
  std::string error;
  FFApplicationService::StatusCallback callback;
  std::atomic<bool> cancelRequested{false};
  bool moving{false};
};
Result<void> FFApplicationService::replaceMachine(
    FFConfig config, bool sim, const std::array<std::string, 3> &serialPorts,
    int activeChannelCount) {
  // The JSON stores editable/default connection values.  Resolve the actual
  // runtime ports first so Simulation and disabled channels are not rejected
  // because of blank or duplicate defaults in the file.
  activeChannelCount = std::clamp(activeChannelCount, 1, 3);
  for (int i = 0; i < 3; ++i) {
    if (sim || i >= activeChannelCount)
      config.channels[i].controller.serial.portName =
          "SIM-CH" + std::to_string(i + 1);
    else
      config.channels[i].controller.serial.portName = serialPorts[i];
  }
  auto valid = FFConfigIO::validate(config);
  if (!valid)
    return valid;
  std::array<std::unique_ptr<IControllerBackend>, 3> backends;
  for (int i = 0; i < 3; ++i)
    backends[i] = (sim || i >= activeChannelCount) ? makeSimulationBackend()
                                                     : makeMightyZapBackend();
  auto machine = std::make_unique<FF>(std::move(config), std::move(backends));
  auto initialized = machine->initialize();
  if (!initialized)
    return initialized;
  impl_->simulation = sim;
  impl_->activeChannelCount = activeChannelCount;
  impl_->machine = std::move(machine);
  impl_->connected = false;
  return Result<void>::success();
}

FFApplicationService::FFApplicationService()
    : impl_(std::make_unique<Impl>()) {}
FFApplicationService::~FFApplicationService() {
  if (impl_->machine)
    impl_->machine->disconnect();
}
Result<void> FFApplicationService::loadConfig(const std::string &file,
                                              bool sim) {
  return loadConfig(file, sim, {"", "", ""});
}
Result<void>
FFApplicationService::loadConfig(const std::string &file, bool sim,
                                 const std::array<std::string, 3> &serialPorts,
                                 int activeChannelCount) {
  auto loaded = FFConfigIO::load(file);
  if (!loaded)
    return Result<void>::failure(loaded.error, loaded.message);
  std::lock_guard l(impl_->mutex);
  if (impl_->connected)
    return Result<void>::failure(ErrorCode::Busy, "Disconnect FF before loading configuration");
  auto x = replaceMachine(std::move(loaded.value), sim, serialPorts,
                          activeChannelCount);
  if (!x) {
    impl_->error = x.message;
    return x;
  }
  return Result<void>::success();
}
Result<void> FFApplicationService::applyConfig(
    const FFConfig &config, bool sim, const std::array<std::string, 3> &ports,
    int activeChannelCount) {
  std::lock_guard l(impl_->mutex);
  if (impl_->connected)
    return Result<void>::failure(ErrorCode::Busy,
                                 "Disconnect FF before applying configuration");
  auto x = replaceMachine(config, sim, ports, activeChannelCount);
  if (!x)
    impl_->error = x.message;
  return x;
}
Result<void> FFApplicationService::connect() {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine)
    return Result<void>::failure(ErrorCode::NotReady,
                                 "No configuration loaded");
  auto x = impl_->machine->connect();
  impl_->connected = static_cast<bool>(x);
  if (!x)
    impl_->error = x.message;
  return x;
}
Result<void> FFApplicationService::disconnect() {
  std::lock_guard l(impl_->mutex);
  if (impl_->moving)
    return Result<void>::failure(ErrorCode::Busy, "Motion is in progress");
  if (impl_->machine)
    impl_->machine->disconnect();
  impl_->connected = false;
  return Result<void>::success();
}
Result<void> FFApplicationService::refresh() {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine)
    return Result<void>::failure(ErrorCode::NotReady);
  auto x = impl_->machine->refreshStatus();
  if (impl_->callback)
    impl_->callback(impl_->machine->status());
  return x;
}
Result<void> FFApplicationService::refreshRod(RodAddress address) {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine)
    return Result<void>::failure(ErrorCode::NotReady);
  auto x = impl_->machine->refreshRodStatus(address);
  if (impl_->callback)
    impl_->callback(impl_->machine->status());
  return x;
}
Result<void> FFApplicationService::setRodTarget(const RodTarget &target) {
  return setRodTargets({target});
}
Result<void>
FFApplicationService::setRodTargets(const std::vector<RodTarget> &targets) {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine || !impl_->connected)
    return Result<void>::failure(ErrorCode::NotConnected);
  // Both simulated and physical backends must go through FF::submitMotion().
  // This is where shared limit, topology, and layer-clearance checks run.
  auto x = impl_->machine->submitMotion({targets, false});
  return x ? Result<void>::success()
           : Result<void>::failure(x.error, x.message);
}
Result<void>
FFApplicationService::validateRodTargets(const std::vector<RodTarget> &targets) {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine || !impl_->connected)
    return Result<void>::failure(ErrorCode::NotConnected);
  auto refreshed = impl_->machine->refreshStatus();
  if (!refreshed)
    return Result<void>::failure(refreshed.error, refreshed.message);
  return impl_->machine->validateMotion({targets, false});
}
Result<void> FFApplicationService::validatePositionSet(const std::vector<RodTarget> &targets) {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine || !impl_->connected)
    return Result<void>::failure(ErrorCode::NotConnected);
  return impl_->machine->validateMotion({targets, false});
}
Result<MotionPlan> FFApplicationService::previewPlan(const MotionRequest &r) {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine)
    return Result<MotionPlan>::failure(ErrorCode::NotReady);
  return MotionPlanner::plan(impl_->machine->config(), impl_->machine->status(),
                             r);
}
Result<void> FFApplicationService::executePlan(const MotionPlan &p) {
  {
    std::lock_guard l(impl_->mutex);
    if (!impl_->machine || !p.valid)
      return Result<void>::failure(ErrorCode::InvalidTarget, "Invalid plan");
    if (p.configVersion != impl_->machine->config().configVersion)
      return Result<void>::failure(ErrorCode::Busy, "Plan has expired");
  }
  std::vector<RodTarget> steps;
  for (const auto &step : p.steps)
    steps.push_back({step.address, step.position});
  return executeGroup(steps);
}
Result<void> FFApplicationService::moveRodAndWait(const RodTarget &target) {
  return executeSteps({target}, true);
}
Result<void> FFApplicationService::executeGroup(
    const std::vector<RodTarget> &targets) {
  std::unique_lock l(impl_->mutex);
  if (!impl_->machine || !impl_->connected)
    return Result<void>::failure(ErrorCode::NotConnected);
  if (impl_->moving)
    return Result<void>::failure(ErrorCode::Busy, "Motion is in progress");
  if (targets.empty())
    return Result<void>::failure(ErrorCode::InvalidTarget, "Empty motion plan");
  impl_->moving = true;
  impl_->cancelRequested.store(false);
  auto finish = [&](Result<void> result, bool stop) {
    if (stop) impl_->machine->stopAll();
    impl_->moving = false;
    return result;
  };
  auto fresh = impl_->machine->refreshStatus();
  if (!fresh) return finish(fresh, false);
  auto checked = MotionPlanner::plan(impl_->machine->config(),
                                     impl_->machine->status(),
                                     {targets, false});
  if (!checked)
    return finish(Result<void>::failure(checked.error, checked.message),
                  false);
  auto submitted = impl_->machine->submitMotion({targets, false});
  if (!submitted)
    return finish(Result<void>::failure(submitted.error, submitted.message),
                  true);
  struct WaitTarget {
    Rod *rod;
    RodConfig config;
    std::int32_t position;
    std::chrono::steady_clock::time_point deadline;
    std::optional<std::chrono::steady_clock::time_point> stableSince;
  };
  std::vector<WaitTarget> pending;
  const auto submittedAt = std::chrono::steady_clock::now();
  for (const auto &target : targets) {
    auto *rod = impl_->machine->findRod(target.address);
    if (!rod)
      return finish(Result<void>::failure(ErrorCode::RodNotFound), true);
    const auto config = rod->config();
    pending.push_back({rod, config, target.position,
                       submittedAt + config.moveTimeout, std::nullopt});
  }
  for (;;) {
    if (impl_->cancelRequested.load())
      return finish(Result<void>::failure(ErrorCode::Busy, "Motion stopped"),
                    true);
    auto refreshed = impl_->machine->refreshStatus();
    if (!refreshed) return finish(refreshed, true);
    const auto now = std::chrono::steady_clock::now();
    bool allStable = true;
    for (auto &item : pending) {
      const auto status = item.rod->status();
      if (status.alarm || status.state == RodState::Fault)
        return finish(Result<void>::failure(ErrorCode::DeviceAlarm,
                                            "Rod fault while moving"), true);
      const bool atTarget = status.positionValid &&
          std::abs(status.actualPosition - item.position) <=
              item.config.positionTolerance;
      if (atTarget) {
        if (!item.stableSince) item.stableSince = now;
      } else {
        item.stableSince.reset();
      }
      if (!item.stableSince || now - *item.stableSince < item.config.stableTime)
        allStable = false;
      if ((!item.stableSince ||
           now - *item.stableSince < item.config.stableTime) &&
          now >= item.deadline)
        return finish(Result<void>::failure(ErrorCode::Timeout,
                                            "Timed out waiting for Rod target"),
                      true);
    }
    if (allStable) return finish(Result<void>::success(), false);
    l.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    l.lock();
  }
}
Result<void> FFApplicationService::executeSteps(const std::vector<RodTarget> &steps,
                                                bool scopedRefresh) {
  std::unique_lock l(impl_->mutex);
  if (!impl_->machine || !impl_->connected)
    return Result<void>::failure(ErrorCode::NotConnected);
  if (impl_->moving)
    return Result<void>::failure(ErrorCode::Busy, "Motion is in progress");
  if (steps.empty())
    return Result<void>::failure(ErrorCode::InvalidTarget, "Empty motion plan");
  impl_->moving = true;
  impl_->cancelRequested.store(false);
  auto finish = [&](Result<void> result) {
    impl_->moving = false;
    return result;
  };
  for (const auto &step : steps) {
    if (impl_->cancelRequested.load())
      return finish(Result<void>::failure(ErrorCode::Busy, "Motion stopped"));
    if (scopedRefresh) {
      auto fresh = impl_->machine->refreshRodStatus(step.address);
      if (!fresh)
        return finish(fresh);
      const auto rodConfig = impl_->machine->findRod(step.address);
      if (!rodConfig)
        return finish(Result<void>::failure(ErrorCode::RodNotFound));
      const auto current = rodConfig->status();
      if (!current.positionValid || current.alarm)
        return finish(Result<void>::failure(ErrorCode::SafetyViolation,
                                            "Rod feedback is invalid"));
      const auto first = std::min(current.actualPosition, step.position);
      const auto last = std::max(current.actualPosition, step.position);
      for (const auto &range : rodConfig->config().forbiddenRanges)
        if (first <= range.second && last >= range.first)
          return finish(Result<void>::failure(ErrorCode::SafetyViolation,
                                              "Motion crosses a forbidden range"));
    }
    auto submitted = impl_->machine->submitMotion({{step}, false});
    if (!submitted)
      return finish(Result<void>::failure(submitted.error, submitted.message));
    const auto *rod = impl_->machine->findRod(step.address);
    if (!rod)
      return finish(Result<void>::failure(ErrorCode::RodNotFound));
    const auto config = rod->config();
    const auto deadline = std::chrono::steady_clock::now() + config.moveTimeout;
    std::optional<std::chrono::steady_clock::time_point> stableSince;
    for (;;) {
      if (impl_->cancelRequested.load())
        return finish(Result<void>::failure(ErrorCode::Busy, "Motion stopped"));
      auto refreshed = scopedRefresh
          ? impl_->machine->refreshRodStatus(step.address)
          : impl_->machine->refreshStatus();
      if (!refreshed)
        return finish(Result<void>::failure(refreshed.error, refreshed.message));
      const auto status = rod->status();
      if (status.alarm || status.state == RodState::Fault)
        return finish(Result<void>::failure(ErrorCode::DeviceAlarm,
                                            "Rod fault while moving"));
      const bool atTarget = status.positionValid &&
          std::abs(status.actualPosition - step.position) <= config.positionTolerance;
      if (atTarget) {
        if (!stableSince) stableSince = std::chrono::steady_clock::now();
        if (std::chrono::steady_clock::now() - *stableSince >= config.stableTime)
          break;
      } else {
        stableSince.reset();
      }
      if (std::chrono::steady_clock::now() >= deadline)
        return finish(Result<void>::failure(ErrorCode::Timeout,
                                            "Timed out waiting for Rod target"));
      l.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      l.lock();
    }
  }
  return finish(Result<void>::success());
}
Result<void> FFApplicationService::cancelMotion() {
  impl_->cancelRequested.store(true);
  std::lock_guard l(impl_->mutex);
  return impl_->machine ? impl_->machine->stopAll() : Result<void>::success();
}
Result<void> FFApplicationService::emergencyStop() {
  impl_->cancelRequested.store(true);
  std::lock_guard l(impl_->mutex);
  return impl_->machine ? impl_->machine->emergencyStop()
                        : Result<void>::success();
}
Result<void> FFApplicationService::resetFaults() {
  std::lock_guard l(impl_->mutex);
  return impl_->machine ? impl_->machine->resetFaults()
                        : Result<void>::failure(ErrorCode::NotReady);
}
FFStatus FFApplicationService::status() const {
  std::lock_guard l(impl_->mutex);
  return impl_->machine ? impl_->machine->status() : FFStatus{};
}
std::optional<FFConfig> FFApplicationService::config() const {
  std::lock_guard l(impl_->mutex);
  return impl_->machine ? std::optional<FFConfig>(impl_->machine->config())
                        : std::nullopt;
}
std::optional<RodConfig>
FFApplicationService::rodConfig(RodAddress address) const {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine)
    return std::nullopt;
  const auto *rod = impl_->machine->findRod(address);
  return rod ? std::optional<RodConfig>(rod->config()) : std::nullopt;
}
bool FFApplicationService::connected() const noexcept {
  return impl_->connected;
}
bool FFApplicationService::simulation() const noexcept {
  return impl_->simulation;
}
int FFApplicationService::activeChannelCount() const noexcept {
  return impl_->activeChannelCount;
}
std::string FFApplicationService::lastError() const { return impl_->error; }
void FFApplicationService::setStatusCallback(StatusCallback c) {
  std::lock_guard l(impl_->mutex);
  impl_->callback = std::move(c);
}
} // namespace ff
