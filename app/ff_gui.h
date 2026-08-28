#pragma once
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "implot.h"
#include "ff/FF.hpp"

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
    std::string notice_;
    bool simulation_{true};
    bool configLoaded_{false};
    void ensureConfig();
    void welcome();
    void dashboard();
    void drawChannel(int index, const ff::FFStatus& status);
};
}
