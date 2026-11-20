#pragma once
#include <d3d11.h>
#include <windows.h>
#pragma comment(lib, "d3d11.lib")
#include "DX_FF.h"
#include "gui_driver.hpp"
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "implot.h"
#include "ImGuiFileDialog.h"
#include <future>

namespace ff_gui {
class MainUi {
public:
  MainUi();
  void draw();
  bool trainingWindowActive() const noexcept {
    return showTraining_;
  }

private:
  ff::GuiDriver service_;
  std::vector<ff::RodTarget> targets_;
  std::string configPath_;
  std::string iniPath_;
  std::string presetDirectory_;
  std::string notice_;
  std::array<std::string, 3> serialPorts_{"COM3", "COM4", "COM5"};
  std::vector<std::string> availablePorts_;
  bool simulation_{true};
  int enabledChannelCount_{1};
  std::array<bool,3> enabledChannels_{true,false,false};
  bool configLoaded_{false};
  bool targetsInitialized_{false};
  int activeChannel_{0};
  int selectedPresetSlot_{0};
  std::vector<int> presetSlots_;
  bool showParameters_{false};
  bool showTraining_{false};
  bool parametersInitialized_{false};
  bool parametersDirty_{false};
  int selectedRod_{0};
  int selectedLayerMember_{0};
  int selectedClearanceRule_{0};
  ff::FFConfig parameterOriginal_{};
  ff::FFConfig parameterDraft_{};
  std::string parameterNotice_;
  std::string pendingLoadPath_;
  std::string trainingDirectory_;
  std::string trainingNotice_;
  std::future<::FF::Result> initialization_;
  ff::FFStatus statusBeforeInitialization_{};
  void beginInitialize();
  ff::TrainingSession training_{};
  std::array<ff::TrainingSession, 3> trainingSessions_{};
  std::vector<ff::RodTarget> trainingExecutionTargets_;
  std::optional<ff::RodTarget> trainingEvasionTarget_;
  bool trainingScanning_{false};
  bool trainingMovePending_{false};
  // Scan-specific dwell after in-tolerance feedback.  This is intentionally
  // independent of RodConfig::stableTime, which is used for full motion plans.
  int trainingSettleMs_{20};
  int trainingSampleCount_{300};
  int trainingSampleIndex_{0};
  int trainingOperationChannel_{-1};
  std::array<std::string,3> trainingPlanSignatures_{};
  std::chrono::steady_clock::time_point trainingStepStarted_{};
  std::array<std::optional<std::chrono::steady_clock::time_point>,3> trainingChannelStable_{};
  double trainingNextMm_{0.0};
  std::optional<std::chrono::steady_clock::time_point> trainingStableSince_;
  std::array<bool, 3> trainingRangeInitialized_{};
  int trainingDraggingCandidate_{-1};
  double trainingDragOriginalMm_{0.0};
  double trainingDragLastSafeMm_{0.0};
  bool trainingForbiddenDirty_{true};
  std::vector<std::pair<double, double>> trainingForbiddenBands_;
  void ensureConfig();
  void loadUiSettings();
  void saveUiSettings() const;
  void scanSerialPorts();
  void refreshPresetSlots();
  bool hasPresetSlot(int slot) const;
  std::string presetPath(int slot) const;
  bool savePreset(int slot, const std::vector<ff::RodTarget> &targets);
  ff::Result<ff::PositionFileData> loadPreset(int slot);
  void welcome();
  void dashboard();
  void drawChannel(int index, const ff::FFStatus &status);
  void parameters();
  void drawParameterSystem();
  void drawParameterController(ff::FFChannelConfig&);
  void drawParameterTraining(ff::FFChannelConfig&);
  void drawParameterRods(ff::FFChannelConfig&, const std::string&);
  void drawParameterLayers(ff::FFChannelConfig&);
  void drawParameterClearance(ff::FFChannelConfig&);
  void training();
  void drawTrainingSettings(const ff::FFConfig&, int, const ff::FFStatus&,
                            std::optional<ff::RodId>, bool&);
  void drawTrainingScanActions(const ff::FFConfig&, int, std::optional<ff::RodId>);
  void initializeTraining();
  void startTrainingScan(const ff::FFConfig&, const std::vector<ff::RodConfig>&);
  void drawTrainingCandidates(const ff::FFConfig&, int, std::optional<ff::RodId>,
                              int, bool, bool&);
  void drawTrainingResults(const ff::FFConfig&, int, const ff::FFStatus&);
  void saveTrainingSession(const ff::FFConfig&);
  void drawTrainingFiles(const ff::FFConfig&);
  void tickTrainingScan();
  void resetTrainingMeasurement();
  bool updateTrainingAssignment();
  bool candidateSetIsLegal(const std::vector<double> &, std::string *reason = nullptr);
  void refreshTrainingForbiddenBands();
  void initializeParameters();
  bool saveParameterFile(const std::string &path);
  bool loadParameterFile(const std::string &path);
  void markParametersDirty();
};
} // namespace ff_gui
