#pragma once

#include "ff/FF.hpp"

namespace ff {

enum class TrainingSource { Sim1, Sim2, Real, Loaded };

struct TrainingSample {
  double positionMm{0.0};
  std::uint64_t energy{0};
};

struct ValleyDetectionSettings {
  int smoothingPoints{5};
  double minimumProminenceFraction{0.10};
  double minimumDistanceMm{0.5};
  int maximumCount{0}; // 0 means no explicit cap.
};

struct TrainingAim {
  RodAddress address{};
  double requestedMm{0.0};
  double quantizedMm{0.0};
  std::int32_t encoderAim{0};
  std::int32_t actualEncoder{0};
  bool assigned{false};
};

// Snapshot of every Rod in the training channel at the moment a session is
// saved.  This records observed state separately from the planned aim.
struct TrainingRodState {
  RodAddress address{};
  std::int32_t actualEncoder{0};
  std::int32_t targetEncoder{0};
};

struct RodAimAssignment {
  std::vector<TrainingAim> aims;
  double totalEncoderTravel{0.0};
  std::string message;
};

struct TrainingDragInterval {
  double minimumMm{0.0};
  double maximumMm{0.0};
  bool valid{false};
};

struct TrainingSession {
  static constexpr const char *Schema = "FF-TRAIN-1";
  TrainingSource source{TrainingSource::Sim1};
  ChannelId channel{ChannelId::CH1};
  RodId trainingRod{0};
  double startMm{0.0}, endMm{0.0}, stepMm{0.1};
  ValleyDetectionSettings valleySettings{};
  std::vector<TrainingSample> samples;
  std::vector<double> candidatesMm;
  std::vector<TrainingAim> aims;
  std::vector<TrainingRodState> rodStates;
  // Human-readable information for identifying the obstruction test without
  // having to inspect the full sample array.
  std::string occlusionSummary;
  // Complete, validated channel target state produced by Train.  Main UI can
  // execute this through the same path as an ordinary position slot.
  std::vector<RodTarget> executionTargets;
  std::string machineId;
  std::string configVersion;
  std::int64_t createdUnixSeconds{0};
  bool complete{false};
};

struct TrainingArchive {
  std::string machineId, configVersion;
  std::int64_t createdUnixSeconds{0};
  int selectedChannel{0};
  std::array<std::optional<TrainingSession>,3> channels{};
};

class TrainingAnalysis {
public:
  static std::uint64_t simulatedEnergy(TrainingSource source, double positionMm,
                                        double startMm, double endMm);
  static double interpolateEnergy(const std::vector<TrainingSample> &samples,
                                  double positionMm);
  static std::vector<double> findValleys(const std::vector<TrainingSample> &,
                                         const ValleyDetectionSettings &);
  // Finds the minimum-total-travel legal one-to-one Rod assignment. For an
  // ordered Layer 2 chain, non-candidate Rods receive safe evasion targets
  // when needed so parked Rods do not block candidate allocation.
  static Result<RodAimAssignment>
  assign(const FFConfig &, const FFStatus &, ChannelId,
         const std::vector<double> &candidateMm,
         std::optional<RodId> excludedRod = std::nullopt);
  // Exact feasibility check for interactive editing.  Unlike assign(), it
  // stops at the first safe one-to-one mapping and does not optimize travel.
  static Result<void> validateAssignment(const FFConfig &, const FFStatus &,
                                         ChannelId,
                                         const std::vector<double> &candidateMm,
                                         std::optional<RodId> excludedRod = std::nullopt);
  static TrainingDragInterval dragInterval(
      const std::vector<double> &candidateMm, std::size_t movingIndex,
      double anchorMm, double scanStartMm, double scanEndMm,
      double windowWidthMm = 3.0);
  static Result<void> saveSession(const std::string &, const TrainingSession &);
  static Result<TrainingSession> loadSession(const std::string &);
  static Result<void> saveArchive(const std::string &, const TrainingArchive &);
  static Result<TrainingArchive> loadArchive(const std::string &);
};

// Hardware acquisition is isolated from the Training UI.  The first release
// deliberately provides only the unavailable Itek adapter because the SDK DLL
// and a validated camera configuration are not bundled with FF.
class IEnergySource {
public:
  virtual ~IEnergySource() = default;
  virtual bool available() const noexcept = 0;
  virtual Result<std::uint64_t> captureEnergy() = 0;
  virtual std::string diagnostic() const = 0;
};

class ItekEnergySource final : public IEnergySource {
public:
  bool available() const noexcept override;
  Result<std::uint64_t> captureEnergy() override;
  std::string diagnostic() const override;
};

const char *trainingSourceName(TrainingSource) noexcept;
} // namespace ff
