#pragma once
#include <d3d11.h>
#include <windows.h>
#pragma comment(lib, "d3d11.lib")
#include "ff/FF.hpp"
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "implot.h"

namespace ff_gui {
class MainUi {
public:
  MainUi();
  void draw();

private:
  ff::FFApplicationService service_;
  ff::MotionPlan plan_;
  std::vector<ff::RodTarget> targets_;
  std::string configPath_;
  std::string iniPath_;
  std::string notice_;
  std::array<std::string, 3> serialPorts_{"COM3", "COM4", "COM5"};
  std::vector<std::string> availablePorts_;
  bool simulation_{true};
  int enabledChannelCount_{3};
  bool configLoaded_{false};
  bool targetsInitialized_{false};
  int activeChannel_{0};
  void ensureConfig();
  void loadUiSettings();
  void saveUiSettings() const;
  void scanSerialPorts();
  void welcome();
  void dashboard();
  void drawChannel(int index, const ff::FFStatus &status);
};
} // namespace ff_gui
