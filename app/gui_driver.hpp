#pragma once

#include "DX_FF.h"
#include "private_training.hpp"

namespace ff {

// GUI-only adapter. All device commands cross the published DX_FF.h boundary;
// the local FFConfig model is used only by the parameter editor and UI layout.
class GuiDriver {
public:
  GuiDriver();
  Result<void> loadConfig(const std::string&, bool,
      const std::array<std::string, 3>&, int);
  Result<void> applyConfig(const FFConfig&, bool,
      const std::array<std::string, 3>&, int);
  Result<void> connect();
  Result<void> disconnect();
  Result<void> refresh();
  Result<void> refreshRod(RodAddress);
  Result<void> setRodTarget(const RodTarget&);
  Result<void> setRodTargets(const std::vector<RodTarget>&);
  Result<void> validateRodTargets(const std::vector<RodTarget>&);
  Result<void> cancelMotion();
  Result<void> emergencyStop();
  FFStatus status() const;
  std::optional<FFConfig> config() const { return config_; }
  std::optional<RodConfig> rodConfig(RodAddress) const;
  bool connected() const noexcept { return connected_; }
  bool simulation() const noexcept { return simulation_; }
  int activeChannelCount() const noexcept { return activeChannels_; }
  ::FF::ff_driver& sdk() noexcept { return driver_; }
  const ::FF::ff_driver& sdk() const noexcept { return driver_; }

private:
  ::FF::ff_driver driver_;
  std::optional<FFConfig> config_;
  std::string configPath_;
  std::array<std::string, 3> ports_{};
  int activeChannels_{3};
  bool simulation_{true};
  bool connected_{false};
};

GuiDriver* activeGuiDriver() noexcept;
::FF::Address publicAddress(RodAddress);
::FF::Target publicTarget(const RodTarget&);
RodTarget privateTarget(const ::FF::Target&);
Result<void> privateResult(const ::FF::Result&);

struct PositionFileData {
  std::string machineId, configVersion, summary;
  std::vector<RodTarget> targets;
};
class GuiPositionFiles {
public:
  static Result<PositionFileData> load(const std::string&,const FFConfig&);
  static Result<void> save(const std::string&,const PositionFileData&);
  static Result<void> validate(const PositionFileData&,const FFConfig&,int);
  static Result<void> execute(GuiDriver&,const PositionFileData&,bool);
};

} // namespace ff

