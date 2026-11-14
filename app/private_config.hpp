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

#if defined(FF_APPLICATION_EXPORTS)
#define FF_SDK_API __declspec(dllexport)
#elif defined(FF_SDK_CONSUMER)
#define FF_SDK_API __declspec(dllimport)
#else
#define FF_SDK_API
#endif

namespace ff {

// Optical-position resolution used when comparing layer-clearance boundaries.
// A measured gap may be this much smaller than the configured minimum without
// being treated as a safety violation.
inline constexpr double kLayerClearanceToleranceMm = 0.10;

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
  // Training Rod is a machine/topology setting, not a per-user UI choice.
  // A missing value explicitly means this channel is not configured for
  // Training yet.
  std::optional<RodId> trainingRod;
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
  static Result<void> validate(const FFConfig &config);
};
} // namespace ff
