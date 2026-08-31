#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ff {

enum class ChannelId : std::uint8_t { CH1 = 0, CH2 = 1, CH3 = 2 };
using RodId = std::uint8_t;

struct RodAddress {
  ChannelId channel{};
  RodId rod{};
  friend constexpr bool operator==(RodAddress a, RodAddress b) noexcept {
    return a.channel == b.channel && a.rod == b.rod;
  }
};

struct RodAddressHash {
  std::size_t operator()(RodAddress a) const noexcept {
    return (static_cast<std::size_t>(a.channel) << 8U) |
           static_cast<std::size_t>(a.rod);
  }
};

enum class FFState {
  Offline,
  Initializing,
  Ready,
  Moving,
  Degraded,
  Fault,
  EmergencyStopped
};
enum class ChannelState { Offline, Connecting, Ready, Moving, Fault };
enum class RodState { Unknown, Disabled, Idle, Moving, Homing, Fault };
enum class SafetyState { Unknown, Safe, Unsafe, Checking, Fault };
enum class SafetyViolation {
  None,
  CommunicationLost,
  PositionUnknown,
  DeviceAlarm,
  HardLimitActive,
  SoftLimitViolation,
  ForbiddenRange,
  RodInterference,
  InvalidTopology,
  StaleFeedback,
  ConfigurationInvalid,
  ExternalUnsafe
};
enum class ErrorCode {
  None,
  InvalidConfiguration,
  MissingChannel,
  DuplicateId,
  DuplicatePort,
  NotConnected,
  NotReady,
  RodNotFound,
  InvalidTarget,
  LimitViolation,
  DuplicateTarget,
  SafetyViolation,
  Busy,
  Timeout,
  DeviceAlarm,
  Communication,
  EmergencyStopped,
  BackendFailure
};

template <typename T> struct Result {
  T value{};
  ErrorCode error{ErrorCode::None};
  std::string message;
  explicit operator bool() const noexcept { return error == ErrorCode::None; }
  static Result success(T v) { return {std::move(v), ErrorCode::None, {}}; }
  static Result failure(ErrorCode e, std::string m = {}) {
    return {T{}, e, std::move(m)};
  }
};

template <> struct Result<void> {
  ErrorCode error{ErrorCode::None};
  std::string message;
  explicit operator bool() const noexcept { return error == ErrorCode::None; }
  static Result success() { return {}; }
  static Result failure(ErrorCode e, std::string m = {}) {
    return {e, std::move(m)};
  }
};

struct SerialConfig {
  std::string portName;
  std::uint32_t baudRate{115200};
  std::chrono::milliseconds timeout{500};
};

struct RodConfig {
  RodId id{};
  std::string name;
  std::int32_t hardMin{0}, hardMax{0};
  std::int32_t softMin{0}, softMax{0};
  std::int32_t zeroPosition{0}, safePosition{0};
  bool safePositionValid{false};
  // optical_position = P0 + P1*u,
  // where u is the encoder position.
  double p0{0.0}, p1{1.0};
  int inwardDirection{1};
  double maxVelocity{0.0}, maxAcceleration{0.0}, maxDeceleration{0.0};
  std::int32_t positionTolerance{0};
  std::chrono::milliseconds stableTime{0}, moveTimeout{5000};
  std::vector<std::pair<std::int32_t, std::int32_t>> absoluteSafeRanges;
  std::vector<std::pair<std::int32_t, std::int32_t>> forbiddenRanges;
};

// A Rod may occur in several records when it participates in several layers.
// Neighbor IDs are -1 at the corresponding end of an open layer.
struct LayerMembership {
  RodId rod{};
  int layer{0};
  int role{0}; // 0: middle, 1: left end, 2: right end, 3: isolated
  int leftNeighbor{-1};
  int rightNeighbor{-1};
};

struct PairClearanceRule {
  int layer{0};
  RodId first{0}, second{0};
  double minimumClearanceMm{0.0};
};

struct ControllerConfig {
  std::string name;
  SerialConfig serial;
  std::chrono::milliseconds pollPeriod{20};
  std::uint32_t retryCount{3};
};

struct FFChannelConfig {
  ChannelId id{};
  ControllerConfig controller;
  std::vector<RodConfig> rods;
  std::vector<LayerMembership> layerMembers;
  std::vector<PairClearanceRule> clearanceRules;
};

struct FFConfig {
  std::string machineId;
  std::string configVersion;
  std::array<FFChannelConfig, 3> channels{};
  std::size_t maxActiveRodCount{0};
  std::chrono::milliseconds statusExpiration{200};
};

struct RodStatus {
  std::int32_t actualPosition{0}, targetPosition{0};
  RodState state{RodState::Unknown};
  bool enabled{false}, positionValid{false}, alarm{false};
  std::uint32_t alarmCode{0};
  std::chrono::steady_clock::time_point updateTime{};
};

struct ChannelStatus {
  ChannelId id{};
  ChannelState state{ChannelState::Offline};
  bool connected{false};
  ErrorCode lastError{ErrorCode::None};
  std::vector<RodStatus> rods;
};

struct ChannelSafetyStatus {
  ChannelId channel{};
  SafetyState state{SafetyState::Unknown};
  std::vector<SafetyViolation> violations;
  std::chrono::steady_clock::time_point checkedAt{};
  bool externallyOverridden{false};
};

struct FFStatus {
  FFState state{FFState::Offline};
  std::array<ChannelStatus, 3> channels{};
  std::array<ChannelSafetyStatus, 3> safety{};
};

struct RodTarget {
  RodAddress address;
  std::int32_t position{0};
  double velocity{0.0};
};

struct MotionRequest {
  std::vector<RodTarget> targets;
  bool allowSoftwareSynchronousStart{true};
};

struct ChannelReceipt {
  ChannelId channel{};
  Result<void> result{};
  std::chrono::steady_clock::time_point submittedAt{};
};

struct MotionReceipt {
  std::vector<ChannelReceipt> channels;
  explicit operator bool() const noexcept {
    for (const auto &r : channels)
      if (!r.result)
        return false;
    return true;
  }
};

enum class PlanStepKind { MoveToSafe, MoveToTarget };
struct MotionPlanStep {
  PlanStepKind kind{PlanStepKind::MoveToTarget};
  RodAddress address{};
  std::int32_t position{0};
  double velocity{0.0};
  std::string reason;
};

struct MotionPlan {
  std::uint64_t id{0};
  std::string configVersion;
  std::vector<MotionPlanStep> steps;
  bool valid{false};
  std::string message;
};

class MotionPlanner {
public:
  static Result<MotionPlan> plan(const FFConfig &, const FFStatus &,
                                 const MotionRequest &);
};

class FFConfigIO {
public:
  // JSON is the canonical on-disk format. Parsing/serialization is
  // implemented in .cpp so the public header has no JSON dependency.
  static Result<FFConfig> load(const std::string &fileName);
  static Result<void> save(const std::string &fileName, const FFConfig &config);
};

// Adapter implemented in .cpp using CppDllMightyZap_Ver2010.h.
class IControllerBackend {
public:
  virtual ~IControllerBackend() = default;
  virtual Result<void> open(const SerialConfig &) = 0;
  virtual Result<void> close() = 0;
  virtual bool isOpen() const noexcept = 0;
  virtual Result<void> setGoal(RodId, std::int32_t, double) = 0;
  // Must use the controller/bus synchronous-write facility when available.
  virtual Result<void> setGoalsSynchronous(const std::vector<RodTarget> &) = 0;
  virtual Result<void> stop(RodId) = 0;
  virtual Result<void> stopAll() = 0;
  virtual Result<void> enable(RodId, bool) = 0;
  virtual Result<void> home(RodId) = 0;
  virtual Result<RodStatus> readStatus(RodId) = 0;
  virtual Result<void> resetAlarm(RodId) = 0;
};

std::unique_ptr<IControllerBackend> makeSimulationBackend();
std::unique_ptr<IControllerBackend> makeMightyZapBackend();

class Rod {
public:
  Rod(RodAddress address, RodConfig config);
  RodAddress address() const noexcept;
  const RodConfig &config() const noexcept;
  RodStatus status() const;
  bool isPositionLegal(std::int32_t position) const noexcept;
  bool isAtTarget() const noexcept;
  double encoderToOpticalPosition(std::int32_t position) const noexcept;
  std::optional<std::int32_t>
  opticalPositionToEncoder(double position) const noexcept;

private:
  RodAddress address_{};
  RodConfig config_{};
  mutable std::mutex mutex_;
  RodStatus status_{};
  friend class Controller;
};

class Controller {
public:
  Controller(ChannelId channel, ControllerConfig config,
             std::unique_ptr<IControllerBackend> backend);
  ~Controller();
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;

  ChannelId channel() const noexcept;
  Result<void> addRod(RodConfig config);
  Rod *findRod(RodId id) noexcept;
  const Rod *findRod(RodId id) const noexcept;
  const std::vector<std::unique_ptr<Rod>> &rods() const noexcept;
  Result<void> connect();
  Result<void> disconnect();
  bool isConnected() const noexcept;
  Result<void> refreshStatus();
  Result<void> enable(RodId, bool);
  Result<void> home(RodId);
  Result<void> submit(RodTarget);
  Result<void> submitSynchronous(const std::vector<RodTarget> &targets);
  Result<void> stop(RodId);
  Result<void> stopAll();
  Result<void> resetFaults();
  ChannelStatus status() const;
  // Evaluates the latest cached motor status and positions without I/O.
  ChannelSafetyStatus evaluateSafety() const;
  // Refreshes motor status through the backend, then evaluates safety.
  Result<ChannelSafetyStatus> refreshAndEvaluateSafety();
  // Explicit maintenance/test override. Hardware alarms and communication
  // failures still make the channel unsafe and cannot be overridden.
  Result<void> setExternalSafetyOverride(std::optional<bool> safe,
                                         std::string reason = {});
  bool isSafe() const noexcept;

private:
  ChannelId channel_{};
  ControllerConfig config_{};
  std::unique_ptr<IControllerBackend> backend_;
  std::vector<std::unique_ptr<Rod>> rods_;
  std::unordered_map<RodId, Rod *> rodsById_;
  mutable std::mutex ioMutex_;
  mutable std::mutex safetyMutex_;
  std::optional<bool> externalSafetyOverride_;
  std::string externalSafetyReason_;
};

class FFChannel {
public:
  explicit FFChannel(FFChannelConfig config,
                     std::unique_ptr<IControllerBackend> backend);
  ~FFChannel();
  FFChannel(const FFChannel &) = delete;
  FFChannel &operator=(const FFChannel &) = delete;
  FFChannel(FFChannel &&) noexcept;
  FFChannel &operator=(FFChannel &&) noexcept;

  ChannelId id() const noexcept;
  Controller &controller() noexcept;
  const Controller &controller() const noexcept;
  ChannelStatus status() const;
  // Checks current safety, then synchronously sends every Rod to its
  // configured safe position. No command is sent if the check fails.
  Result<void> moveAllToSafePositions();
  // Checks current and target safety, then synchronously sends every Rod to
  // the caller-supplied positions. The target list must contain every Rod
  // in this Channel exactly once.
  Result<void> moveAllToPositions(const std::vector<RodTarget> &targets);

private:
  ChannelId id_{};
  std::unique_ptr<Controller> controller_;
};

class FF {
public:
  explicit FF(FFConfig config,
              std::array<std::unique_ptr<IControllerBackend>, 3> backends);
  ~FF();
  FF(const FF &) = delete;
  FF &operator=(const FF &) = delete;

  Result<void> initialize();
  static Result<FFConfig> loadConfig(const std::string &fileName);
  Result<void> saveConfig(const std::string &fileName) const;
  Result<void> applyConfig(FFConfig config);
  const FFConfig &config() const noexcept;
  Result<void> connect();
  Result<void> disconnect();
  Result<void> refreshStatus();
  Result<ChannelSafetyStatus> refreshChannelSafety(ChannelId id);
  ChannelSafetyStatus channelSafety(ChannelId id) const;
  Result<MotionReceipt> submitMotion(const MotionRequest &request);
  Result<void> moveChannelToSafePositions(ChannelId id);
  Result<void> moveChannelToPositions(ChannelId id,
                                      const std::vector<RodTarget> &targets);
  Result<void> stopAll();
  Result<void> emergencyStop();
  Result<void> resetFaults();

  FFState state() const noexcept;
  FFStatus status() const;
  FFChannel *channel(ChannelId id) noexcept;
  const FFChannel *channel(ChannelId id) const noexcept;
  Rod *findRod(RodAddress address) noexcept;
  const Rod *findRod(RodAddress address) const noexcept;
  Result<void> validateConfiguration() const;
  std::string lastError() const;

private:
  Result<void> validateMotion(const MotionRequest &) const;
  FFConfig config_{};
  std::array<std::unique_ptr<FFChannel>, 3> channels_{};
  mutable std::mutex mutex_;
  FFState state_{FFState::Offline};
  std::string lastError_;
};

class FFApplicationService {
public:
  using StatusCallback = std::function<void(const FFStatus &)>;
  FFApplicationService();
  ~FFApplicationService();
  FFApplicationService(const FFApplicationService &) = delete;
  Result<void> loadConfig(const std::string &fileName, bool simulation);
  Result<void> loadConfig(const std::string &fileName, bool simulation,
                          const std::array<std::string, 3> &serialPorts,
                          int activeChannelCount = 3);
  Result<void> connect();
  Result<void> disconnect();
  Result<void> refresh();
  Result<MotionPlan> previewPlan(const MotionRequest &);
  Result<void> executePlan(const MotionPlan &);
  Result<void> cancelMotion();
  Result<void> emergencyStop();
  FFStatus status() const;
  std::optional<RodConfig> rodConfig(RodAddress address) const;
  bool connected() const noexcept;
  bool simulation() const noexcept;
  int activeChannelCount() const noexcept;
  std::string lastError() const;
  void setStatusCallback(StatusCallback);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace ff
