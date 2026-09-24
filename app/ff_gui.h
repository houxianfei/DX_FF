#pragma once
#include <d3d11.h>
#include <windows.h>
#pragma comment(lib, "d3d11.lib")
#include "ff/FF.hpp"
#include "ff/Training.hpp"
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "implot.h"
#include "ImGuiFileDialog.h"

namespace ff_gui {
class MainUi {
public:
  MainUi();
  void draw();

private:
  ff::FFApplicationService service_;
  std::vector<ff::RodTarget> targets_;
  std::string configPath_;
  std::string iniPath_;
  std::string presetDirectory_;
  std::string notice_;
  std::array<std::string, 3> serialPorts_{"COM3", "COM4", "COM5"};
  std::vector<std::string> availablePorts_;
  bool simulation_{true};
  int enabledChannelCount_{3};
  bool configLoaded_{false};
  bool targetsInitialized_{false};
  int activeChannel_{0};
  int selectedPresetSlot_{0};
  std::vector<int> presetSlots_;
  bool showParameters_{false};
  bool showTraining_{false};
  bool parametersInitialized_{false};
  bool parametersDirty_{false};
  int parameterChannel_{0};
  int selectedRod_{0};
  int selectedLayerMember_{0};
  int selectedClearanceRule_{0};
  ff::FFConfig parameterOriginal_{};
  ff::FFConfig parameterDraft_{};
  std::string parameterNotice_;
  std::string pendingLoadPath_;
  std::string trainingDirectory_;
  std::string trainingNotice_;
  ff::TrainingSession training_{};
  std::array<ff::TrainingSession, 3> trainingSessions_{};
  std::vector<ff::RodTarget> trainingExecutionTargets_;
  std::optional<ff::RodTarget> trainingEvasionTarget_;
  bool trainingScanning_{false};
  bool trainingMovePending_{false};
  // Scan-specific dwell after in-tolerance feedback.  This is intentionally
  // independent of RodConfig::stableTime, which is used for full motion plans.
  int trainingSettleMs_{20};
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
  std::vector<ff::RodTarget> loadPreset(int slot) const;
  void welcome();
  void dashboard();
  void drawChannel(int index, const ff::FFStatus &status);
  void parameters();
  void training();
  void tickTrainingScan();
  void resetTrainingMeasurement();
  bool updateTrainingAssignment();
  bool candidateSetIsLegal(const std::vector<double> &, std::string *reason = nullptr) const;
  void refreshTrainingForbiddenBands();
  void initializeParameters();
  bool saveParameterFile(const std::string &path);
  bool loadParameterFile(const std::string &path);
  void markParametersDirty();
};
} // namespace ff_gui
