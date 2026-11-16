#include "ff/FF.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include "CppDllMightyZap_Ver2010.h"
#include <windows.h>

namespace ff {
class SimulatedControllerBackend final : public IControllerBackend {
  bool open_{false};
  std::mutex mutex_;
  std::unordered_map<RodId, RodStatus> rods_;

public:
  Result<void> configureRods(const std::vector<RodConfig> &configs) override {
    std::vector<RodConfig> ordered = configs;
    std::sort(ordered.begin(), ordered.end(),
              [](const auto &a, const auto &b) { return a.id < b.id; });
    rods_.clear();
    const auto now = std::chrono::steady_clock::now();
    for (size_t index = 0; index < ordered.size(); ++index) {
      const auto &config = ordered[index];
      const auto minimum = (std::max)(config.hardMin, config.softMin);
      const auto maximum = (std::min)(config.hardMax, config.softMax);
      if (minimum > maximum)
        return Result<void>::failure(ErrorCode::InvalidConfiguration,
                                     "Simulation Rod has no legal position");
      const double fraction = ordered.size() <= 1
                                  ? 0.5
                                  : static_cast<double>(index) /
                                        static_cast<double>(ordered.size() - 1);
      const auto position = static_cast<int32_t>(std::lround(
          minimum + (maximum - minimum) * fraction));
      auto &status = rods_[config.id];
      status.actualPosition = position;
      status.targetPosition = position;
      status.positionValid = true;
      status.enabled = true;
      status.state = RodState::Idle;
      status.updateTime = now;
    }
    return Result<void>::success();
  }
  Result<void> open(const SerialConfig &) override {
    open_ = true;
    return Result<void>::success();
  }
  Result<void> close() override {
    open_ = false;
    return Result<void>::success();
  }
  bool isOpen() const noexcept override { return open_; }
  Result<void> setGoal(RodId id, int32_t p) override {
    if (!open_)
      return Result<void>::failure(ErrorCode::NotConnected);
    auto &s = rods_[id];
    s.targetPosition = p;
    s.actualPosition = p;
    s.positionValid = true;
    s.state = RodState::Idle;
    s.enabled = true;
    s.updateTime = std::chrono::steady_clock::now();
    return Result<void>::success();
  }
  Result<void> setGoalsSynchronous(const std::vector<RodTarget> &ts) override {
    for (auto &t : ts) {
      auto x = setGoal(t.address.rod, t.position);
      if (!x)
        return x;
    }
    return Result<void>::success();
  }
  Result<void> stop(RodId id) override {
    rods_[id].state = RodState::Idle;
    return Result<void>::success();
  }
  Result<void> stopAll() override {
    for (auto &x : rods_)
      x.second.state = RodState::Idle;
    return Result<void>::success();
  }
  Result<void> enable(RodId id, bool e) override {
    rods_[id].enabled = e;
    return Result<void>::success();
  }
  Result<void> home(RodId id) override { return setGoal(id, 0); }
  Result<RodStatus> readStatus(RodId id) override {
    if (!open_)
      return Result<RodStatus>::failure(ErrorCode::NotConnected);
    auto &s = rods_[id];
    if (!s.positionValid) {
      s.actualPosition = 0;
      s.targetPosition = 0;
      s.enabled = true;
      s.state = RodState::Idle;
    }
    s.positionValid = true;
    s.updateTime = std::chrono::steady_clock::now();
    return Result<RodStatus>::success(s);
  }
  Result<void> resetAlarm(RodId id) override {
    rods_[id].alarm = false;
    rods_[id].alarmCode = 0;
    return Result<void>::success();
  }
};

#ifdef _WIN32
class MightyZapControllerBackend final : public IControllerBackend {
  MightyZap zap_;
  bool open_{false};
  std::mutex mutex_;
  std::unordered_set<RodId> ids_;
  std::vector<RodConfig> configs_;

public:
  Result<void> configureRods(const std::vector<RodConfig> &configs) override {
    configs_ = configs;
    ids_.clear();
    for (const auto &config : configs) {
      ids_.insert(config.id);
    }
    return Result<void>::success();
  }
  Result<void> open(const SerialConfig &c) override {
    std::wstring w(c.portName.begin(), c.portName.end());
    if (zap_.OpenMightyZap(w.c_str(), c.baudRate)) {
      open_ = true;
      zap_.SerialTimeout(1, 0, 1, 1, 1);
      const auto failConfiguration = [this](std::string message) {
        zap_.CloseMightyZap();
        open_ = false;
        return Result<void>::failure(ErrorCode::Communication, message);
      };
      for (const auto &rod : configs_) {
        const int id = rod.id;
        const short speed = static_cast<short>(rod.maxVelocity);
        const auto acceleration = static_cast<unsigned char>(rod.maxAcceleration);
        const auto deceleration = static_cast<unsigned char>(rod.maxDeceleration);
        short currentShort = -1, currentLong = -1;
        zap_.GetShortStrokeLimit(id, currentShort);
        zap_.GetLongStrokeLimit(id, currentLong);
        if (currentShort < 0 || currentLong < 0 || currentShort > currentLong)
          return failConfiguration("Unable to read motor stroke limits for Rod " +
                                   std::to_string(id));
        if (rod.hardMin < currentShort || rod.hardMax > currentLong)
          return failConfiguration("Configured stroke limits would expand motor travel for Rod " +
                                   std::to_string(id) + "; inspect physical limits");
        short current = -1;
        if (currentShort != rod.hardMin)
          zap_.SetShortStrokeLimit(id, static_cast<short>(rod.hardMin));
        current = -1;
        zap_.GetShortStrokeLimit(id, current);
        if (current != rod.hardMin)
          return failConfiguration("Unable to configure motor short stroke limit for Rod " + std::to_string(id));
        current = -1;
        if (currentLong != rod.hardMax)
          zap_.SetLongtStrokeLimit(id, static_cast<short>(rod.hardMax));
        current = -1;
        zap_.GetLongStrokeLimit(id, current);
        if (current != rod.hardMax)
          return failConfiguration("Unable to configure motor long stroke limit for Rod " + std::to_string(id));
        current = -1;
        zap_.Read_Addr(id, 21, 2, current);
        if (current != speed)
          zap_.Write_Addr(id, 21, 2, speed);
        current = -1;
        zap_.Read_Addr(id, 21, 2, current);
        if (current != speed)
          return failConfiguration("Unable to configure motor speed limit for Rod " + std::to_string(id));
        current = -1;
        zap_.Read_Addr(id, 33, 1, current);
        if (current != acceleration)
          zap_.Acceleration(id, acceleration);
        current = -1;
        zap_.Read_Addr(id, 33, 1, current);
        if (current != acceleration)
          return failConfiguration("Unable to configure motor acceleration for Rod " + std::to_string(id));
        current = -1;
        zap_.Read_Addr(id, 34, 1, current);
        if (current != deceleration)
          zap_.Deceleration(id, deceleration);
        current = -1;
        zap_.Read_Addr(id, 34, 1, current);
        if (current != deceleration)
          return failConfiguration("Unable to configure motor deceleration for Rod " + std::to_string(id));
        zap_.GoalSpeed(id, speed);
      }
      return Result<void>::success();
    }
    return Result<void>::failure(ErrorCode::Communication,
                                 "MightyZap failed to open serial port");
  }
  Result<void> close() override {
    if (open_)
      zap_.CloseMightyZap();
    open_ = false;
    return Result<void>::success();
  }
  bool isOpen() const noexcept override { return open_; }
  Result<void> setGoal(RodId id, int32_t p) override {
    if (!open_)
      return Result<void>::failure(ErrorCode::NotConnected);
    ids_.insert(id);
    zap_.GoalPosition(id, static_cast<short>(p));
    return Result<void>::success();
  }
  Result<void> setGoalsSynchronous(const std::vector<RodTarget> &ts) override {
    if (!open_)
      return Result<void>::failure(ErrorCode::NotConnected);
    for (auto &t : ts) {
      ids_.insert(t.address.rod);
      zap_.GoalPosition(t.address.rod, static_cast<short>(t.position));
    }
    return Result<void>::success();
  }
  Result<void> stop(RodId id) override {
    if (!open_)
      return Result<void>::failure(ErrorCode::NotConnected);
    zap_.ForceEnable(id, false);
    return Result<void>::success();
  }
  Result<void> stopAll() override {
    for (auto id : ids_)
      zap_.ForceEnable(id, false);
    return Result<void>::success();
  }
  Result<void> enable(RodId id, bool e) override {
    if (!open_)
      return Result<void>::failure(ErrorCode::NotConnected);
    ids_.insert(id);
    zap_.ForceEnable(id, e);
    return Result<void>::success();
  }
  Result<void> home(RodId id) override { return setGoal(id, 0); }
  Result<RodStatus> readStatus(RodId id) override {
    if (!open_)
      return Result<RodStatus>::failure(ErrorCode::NotConnected);
    ids_.insert(id);
    short p = 0;
    unsigned char a = 0;
    zap_.PresentPosition(id, p);
    zap_.ReadError(id, a);
    RodStatus s;
    s.actualPosition = p;
    s.targetPosition = p;
    s.positionValid = true;
    s.enabled = true;
    s.alarm = a != 0;
    s.alarmCode = a;
    s.state = s.alarm ? RodState::Fault : RodState::Idle;
    s.updateTime = std::chrono::steady_clock::now();
    return Result<RodStatus>::success(s);
  }
  Result<void> resetAlarm(RodId id) override {
    if (!open_)
      return Result<void>::failure(ErrorCode::NotConnected);
    zap_.reset_write(id, 0);
    return Result<void>::success();
  }
};
#endif

std::unique_ptr<IControllerBackend> makeSimulationBackend() {
  return std::make_unique<SimulatedControllerBackend>();
}
std::unique_ptr<IControllerBackend> makeMightyZapBackend() {
#ifdef _WIN32
  return std::make_unique<MightyZapControllerBackend>();
#else
  return nullptr;
#endif
}
} // namespace ff
