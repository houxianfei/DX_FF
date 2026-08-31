#include "ff/FF.hpp"
#include <algorithm>
#include <mutex>

namespace ff {
struct FFApplicationService::Impl {
  mutable std::mutex mutex;
  std::unique_ptr<FF> machine;
  bool simulation{true};
  int activeChannelCount{3};
  bool connected{false};
  std::string error;
  FFApplicationService::StatusCallback callback;
};
Result<void> FFApplicationService::replaceMachine(
    FFConfig config, bool sim, const std::array<std::string, 3> &serialPorts,
    int activeChannelCount) {
  auto valid = FFConfigIO::validate(config);
  if (!valid)
    return valid;
  activeChannelCount = std::clamp(activeChannelCount, 1, 3);
  for (int i = 0; i < 3; ++i) {
    if (sim || i >= activeChannelCount)
      config.channels[i].controller.serial.portName = "SIM-CH" + std::to_string(i + 1);
    else if (!serialPorts[i].empty())
      config.channels[i].controller.serial.portName = serialPorts[i];
  }
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
Result<MotionPlan> FFApplicationService::previewPlan(const MotionRequest &r) {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine)
    return Result<MotionPlan>::failure(ErrorCode::NotReady);
  return MotionPlanner::plan(impl_->machine->config(), impl_->machine->status(),
                             r);
}
Result<void> FFApplicationService::executePlan(const MotionPlan &p) {
  std::lock_guard l(impl_->mutex);
  if (!impl_->machine || !p.valid)
    return Result<void>::failure(ErrorCode::InvalidTarget, "Invalid plan");
  if (p.configVersion != impl_->machine->config().configVersion)
    return Result<void>::failure(ErrorCode::Busy, "Plan has expired");
  for (auto &s : p.steps) {
    auto x = impl_->machine->submitMotion(
        {{{s.address, s.position, s.velocity}}, false});
    if (!x)
      return Result<void>::failure(x.error, x.message);
  }
  return Result<void>::success();
}
Result<void> FFApplicationService::cancelMotion() {
  std::lock_guard l(impl_->mutex);
  return impl_->machine ? impl_->machine->stopAll() : Result<void>::success();
}
Result<void> FFApplicationService::emergencyStop() {
  std::lock_guard l(impl_->mutex);
  return impl_->machine ? impl_->machine->emergencyStop()
                        : Result<void>::success();
}
FFStatus FFApplicationService::status() const {
  std::lock_guard l(impl_->mutex);
  return impl_->machine ? impl_->machine->status() : FFStatus{};
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
