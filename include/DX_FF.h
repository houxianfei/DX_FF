#pragma once

// Value types, result codes and status snapshots: see DX_FF_Types.h.
#include "DX_FF_Types.h"
#include <functional>
#include <memory>

#if defined(FF_APPLICATION_EXPORTS)
#define DX_FF_API __declspec(dllexport)
#else
#define DX_FF_API __declspec(dllimport)
#endif

namespace FF {

class DX_FF_API ff_driver {
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Result report(Result result);
  void update_status();
  void update_current_positions();

public:
  using StatusCallback = std::function<void(const DeviceStatus&)>;

  // SDK-owned, read-only snapshots. References live as long as this driver;
  // their contents may change after any operation or status callback.
  const ConfigSnapshot& m_config;
  const DeviceStatus& m_status;
  const Result& m_last_error;
  const std::vector<RodPosition>& m_current_positions;
  const std::vector<TargetPosition>& m_target_positions;
  const std::string& m_position_summary;

  ff_driver();
  ~ff_driver();
  ff_driver(const ff_driver&) = delete;
  ff_driver& operator=(const ff_driver&) = delete;

  // Connection and feedback.
  Result open(const std::string& configPath, Mode mode,
              const std::array<std::string, 3>& ports = {}, int activeChannels = 3);
  Result initialize(const std::string& slot0Path);
  Result close();
  Result refresh();
  Result refresh_rod(Address address);
  MachineInfo machine_info() const;
  void set_status_callback(StatusCallback callback);

  // Position files and pending targets. Loading or editing does not move Rods.
  Result load_positions(const std::string& path);
  Result record_positions(const std::string& path, const std::string& summary = {});
  Result set_target_position(Address address, std::int32_t encoder, double velocity);
  Result set_target_positions(const std::vector<Target>& targets);
  Result validate_target_positions(const std::vector<Target>& targets);

  // Planning does not move. send_positions() does not wait; execute_positions()
  // and move_rod() wait for completion.
  ValueResult<Plan> plan_positions();
  Result send_positions(); // Submit the complete pending set without waiting.
  Result execute_positions();
  Result move_rod(Address address, std::int32_t encoder, double velocity);
  Result send_rod(Address address, std::int32_t encoder, double velocity);
  Result stop();
  Result emergency_stop();
  Result reset_faults();

  // Training calculation. Samples and candidates use optical millimetres.
  ValueResult<Assignment> assign_training(int channel,
      const std::vector<double>& candidatesMm, int excludedRod = -1);
  std::uint64_t simulated_energy(TrainingSource source, double positionMm,
                                 double startMm, double endMm);
  double interpolate_energy(const std::vector<Sample>& samples, double positionMm);
  std::vector<double> find_valleys(const std::vector<Sample>& samples,
                                  const ValleySettings& settings);
  DragInterval drag_interval(const std::vector<double>& candidatesMm,
      std::size_t movingIndex, double anchorMm, double scanStartMm,
      double scanEndMm, double windowWidthMm = 3.0);
  Result save_session(const std::string& path, const std::string& sessionJson);
  ValueResult<std::string> load_session(const std::string& path);
  Result save_training_session(const std::string& path, const TrainingSession& session);
  ValueResult<TrainingSession> load_training_session(const std::string& path);
  const char* version() const noexcept;
};

} // namespace FF
