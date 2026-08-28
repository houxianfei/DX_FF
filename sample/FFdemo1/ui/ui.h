#pragma once

#include "../ff_gui.h"
#include "../FF/zap.h"
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include "ff_parameters.h"
#include "../FF/ItekCamera.h"


using namespace std;


namespace	FF {

	//struct ui_welcome_config {
	//	LPCTSTR com_port = "COM4";
	//};


	class ui {
	private:
		ZAP zap;
		ImVector<ImVec2> training_curve_rdata;
		bool show_ui[4] = { false, false, false ,false };
	public:
		bool open_status = false;
		void Welcome();
		void main_ui();
		void training(bool* p_open);
	};
}

