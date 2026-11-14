#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace FF {

// Result codes describe the public driver operation, not individual Rod alarms.
enum class ErrorCode {
  None,
  InvalidConfiguration,
  NotOpen,
  InvalidTarget,
  SafetyViolation,
  DeviceAlarm,
  Communication,
  Timeout,
  Busy,
  BackendFailure
};

struct Result {
  ErrorCode code{ErrorCode::None};
  std::string message;
  explicit operator bool() const noexcept { return code == ErrorCode::None; }
};

template <class T> struct ValueResult {
  T value{};
  Result result{};
  explicit operator bool() const noexcept { return static_cast<bool>(result); }
};

enum class Mode { Simulation, Hardware };

// channel is 0..2; rod is the configured Rod ID, not its vector index.
struct Address {
  int channel{0};
  int rod{0};
};

// Encoder target and configured velocity unit.
struct Target {
  Address address;
  std::int32_t position{0};
  double velocity{0};
};

struct RodPosition {
  Address address;
  std::int32_t encoder{0};
  double opticalMm{0};
  bool valid{false}; // Whether the latest position feedback is usable.
};

struct TargetPosition {
  Address address;
  std::int32_t encoder{0};
  double opticalMm{0};
  bool valid{false};
  double velocity{0};
};

// Hardware feedback target is distinct from the driver's pending target.
struct RodStatus {
  Address address;
  std::int32_t actual{0}, target{0};
  bool positionValid{false}, enabled{false}, alarm{false};
  std::uint32_t alarmCode{0};
};

struct ChannelStatus {
  int channel{0};
  bool connected{false}, safe{false};
  std::vector<RodStatus> rods;
  std::vector<std::string> violations;
};

struct DeviceStatus {
  std::string state; // Offline, Ready, Moving, Fault, EmergencyStopped or Other.
  std::array<ChannelStatus, 3> channels{};
};

struct ConfigSnapshot {
  std::string machineId, configVersion, json;
};

struct RodInfo {
  Address address;
  std::string name;
  std::int32_t hardMin{0}, hardMax{0};
  std::int32_t softMin{0}, softMax{0};
  double p0{0}, p1{1}, maxVelocity{0};
  std::int32_t positionTolerance{0};
};

struct LayerInfo {
  int rod{0}, layer{0}, role{0};
  int leftNeighbor{-1}, rightNeighbor{-1};
};

struct ClearanceRuleInfo {
  int layer{0}, firstRod{0}, secondRod{0};
  double minimumClearanceMm{0};
};

struct ChannelInfo {
  int channel{0};
  int trainingRod{-1};
  std::vector<RodInfo> rods;
  std::vector<LayerInfo> layerMembers;
  std::vector<ClearanceRuleInfo> clearanceRules;
};

struct MachineInfo {
  std::array<ChannelInfo, 3> channels{};
  int activeChannels{0};
};

struct PlanStep {
  Target target;
  bool safePosition{false};
  std::string reason;
};

struct Plan {
  std::string configVersion;
  std::vector<PlanStep> steps;
};

struct Sample {
  double positionMm{0};
  std::uint64_t energy{0};
};

struct ValleySettings {
  int smoothingPoints{5};
  double minimumProminenceFraction{0.1};
  double minimumDistanceMm{0.5};
  int maximumCount{0}; // Zero means no explicit count limit.
};

struct Aim {
  Address address;
  double requestedMm{0}, quantizedMm{0};
  std::int32_t encoderAim{0};
  std::int32_t actualEncoder{0};
  bool assigned{false};
};

struct Assignment {
  std::vector<Aim> aims;
  double totalEncoderTravel{0};
};

struct DragInterval {
  double minimumMm{0}, maximumMm{0};
  bool valid{false};
};

enum class TrainingSource { Sim1, Sim2, Real, Loaded };

struct TrainingRodState {
  Address address;
  std::int32_t actualEncoder{0}, targetEncoder{0};
};

struct TrainingSession {
  TrainingSource source{TrainingSource::Sim1};
  int channel{0}, trainingRod{0};
  double startMm{0}, endMm{0}, stepMm{0.1};
  ValleySettings valleySettings{};
  std::vector<Sample> samples;
  std::vector<double> candidatesMm;
  std::vector<Aim> aims;
  std::vector<TrainingRodState> rodStates;
  std::string occlusionSummary;
  std::vector<Target> executionTargets;
  std::string machineId, configVersion;
  std::int64_t createdUnixSeconds{0};
  bool complete{false};
};

} // namespace FF
