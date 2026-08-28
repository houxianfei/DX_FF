#include "ui.h"

std::vector<std::string> EnumerateAvailableSerialPorts();
std::wstring to_wstring(const std::string& str);
extern bool g_bExit;
extern uint64_t sum_pixels;
void get_rdata_ylim(ImVector<ImVec2> rdata, float& min_y, float& max_y);


void FF::ui::Welcome(){
	static std::vector<std::string> ports;
	static std::string select_port;
	static bool select_flags[3] = { false, false, false };

	if (ImGui::BeginMenuBar())
	{
		if (ImGui::BeginMenu("Connection: ")) {
			ImGui::MenuItem("select communication interface and open", NULL);
			ImGui::EndMenu();
		}
		ImGui::EndMenuBar();
	}

	ImGui::Text("Interface: USB <-> RS485 ");
	ImGui::SameLine();
	if(ImGui::Button("search")) {
		ports = EnumerateAvailableSerialPorts();
	}

	if (!ports.empty()) {
		for (int i = 0; i < ports.size(); i++) {
			auto port = ports[i];
			ImGui::Selectable(port.c_str(), &select_flags[i]);
			if (select_flags[i]) {
				select_port = port;
			}
		}
	}

	if (!select_port.empty()) {
		if (ImGui::Button("open")) {
			std::wstring wport = to_wstring(select_port);
			LPCTSTR com_port = wport.c_str();
			if (zap.OpenMightyZap(com_port, CBR_57600)) {
				printf("mighty_zap open with %s\n", select_port.c_str());
				zap.SerialTimeout(1, 0, 1, 1, 1);
				open_status = true;
			}
			else {
				printf("mighty_zap open failed by \n", select_port.c_str());
				open_status = false;
				zap.CloseMightyZap();
			}
		}
	}



}


void FF::ui::main_ui() {
	static bool save_flag = false;
	if (ImGui::BeginMenuBar())
	{
		if (ImGui::BeginMenu("Tools"))
		{
			ImGui::MenuItem("zeros position calibration", NULL, &show_ui[0]);
			ImGui::MenuItem("Training", NULL, &show_ui[1]);
			ImGui::EndMenu();
		}
		ImGui::EndMenuBar();
	}

	if (show_ui[1]) {
		training(&show_ui[1]);
	}


	ImGui::Text("RS485 is connected. ");
	ImGui::SameLine();
	if (ImGui::Button("close")) {
		zap.CloseMightyZap();
		open_status = false;
		printf("mighty_zap closed.\n");
	}

	ImGui::SameLine();

	if (ImGui::Button("scan device")) {
		zap.scan_sevice();
	}

	ImGui::SeparatorText("Control");
	if (zap.m_rod_num > 0) {
		if (ImGui::Button("save")) {
			zap.record_position();
			save_flag = true;
		} ImGui::SameLine();

		if (ImGui::Button("load") && save_flag) {
			zap.load_record_position();
		}
	}
	for (int i = 0; i < zap.m_rod_num; i++) {
		int bID = zap.m_bIDs[i];
		string bID_string = "stroke" + to_string(bID);
		if (ImGui::SliderFloat(bID_string.c_str(), &zap.m_position[bID], 0.0, FF_RANGE)) {
			if ((bID == 0 || bID == 1 || bID == 13)) {
				zap.setPosition(bID, zap.m_position[bID], true);
			}
			else {
				zap.setPosition(bID, zap.m_position[bID]);
			}
		}
	}
}

void FF::ui::training(bool* p_open) {
	static bool camera_ready = false;
	//static bool initial_ok = false;
	static ItkCamera cam;
	static bool reader_running = false;
	static float start_pos = 5.0f;
	static float move_step = 0.05f;
	static float end_pos = 15.0f;
	static int meas_cnt = 0;

	if (training_curve_rdata.Size == 0) {
		training_curve_rdata.push_back(ImVec2(0.0f, 0.0f));
	}


	/// \~chinese IKapC 函数返回值				    \~english Return value of IKapC functions
	static ITKSTATUS res = ITKSTATUS_OK;

	/// \~chinese IKapBoard 函数返回值				\~english Return value of IKapBoard functions
	static int ret = IK_RTN_OK;

	if (!camera_ready) {

		/// \~chinese 初始化 IKapC 运行环境				\~english Initialize IKapC runtime environment
		res = ItkManInitialize();
		CHECK_IKAPC(res);

		cam.g_bufferCount = 10;

		sprintf_s(cam.g_saveFileName, "D:\\CImage.tif");

		cam.g_SerialNumber = NULL;

		cam.g_bSoftTriggerUsed = 0;

		cam.g_bLoadGrabberConfig = 0;

		uint32_t numDevices = 1;
		//res = ItkManGetDeviceCount(&numDevices);
		//CHECK(res);

		// 当没有连接的设备时。
		//
		// When there is no connected devices.
		//if (numDevices == 0) {
		//	fprintf(stderr, "No device.\n");
		//	ItkManTerminate();
		//	pressEnterToExit();
		//	exit(EXIT_FAILURE);
		//}

		//printCameraInfo(numDevices);

		/// \~chinese 打开序号为0的相机并配置相机设备		\~english Open camera with 0 index and configure camera device
		int tmpIndex = 0;
		//fprintf(stderr, "total device count: %d, the index of which you want to open is:", numDevices);
		//fflush(stderr);
		//int scanTmp = scanf_s("%u", &tmpIndex);
		//if (scanTmp != 1) {
		//	fprintf(stderr, "Invalid input. Expect an integer.\n");
		//	ItkManTerminate();
		//	pressEnterToExit();
		//	exit(EXIT_FAILURE);
		//}

		if (tmpIndex >= numDevices)
		{
			//fprintf(stderr, "index: %d is more than %d.\n", tmpIndex, numDevices - 1);
			ItkManTerminate();
			pressEnterToExit();
			exit(EXIT_FAILURE);
		}

		//{ int getCharTmp = getchar(); }

		ConfigureCamera(&cam, tmpIndex);

		if (cam.g_hBoard == INVALID_HANDLE_VALUE) {
			//fprintf(stderr, "Please select camera with grabber.\n");
			ItkManTerminate();
			pressEnterToExit();
			exit(EXIT_FAILURE);
		}

		/// \~chinese 配置采集卡设备				        \~english Configure frame grabber device
		ConfigureFrameGrabber(&cam);

		/// \~chinese 注册回调函数                        \~english Register callback functions.
		RegisterCallbackWithGrabber(&cam);

		/// \~chinese 配置相机触发方式		            \~english Configure trigger method of the camera
		SetSoftTriggerWithGrabber(&cam);
	
		camera_ready = true;
	}


	if (!ImGui::Begin("training", p_open))
	{
		camera_ready = false;
		/// \~chinese 清除回调函数				        \~english Unregister callback functions
		UnRegisterCallbackWithGrabber(&cam);

		/// \~chinese 关闭采集卡设备				        \~english Close frame grabber device
		ret = IKapClose(cam.g_hBoard);
		CHECK_IKAPBOARD(ret);

		/// \~chinese 关闭相机设备				        \~english Close camera device
		res = ItkDevClose(cam.g_hCamera);
		CHECK_IKAPC(res);

		/// \~chinese 释放 IKapC 运行环境				\~english Release IKapC runtime environment
		ItkManTerminate();

		ImGui::End();
		return;
	}


	ImGui::Text("training");

	if (ImGui::Button("initial")) {
		// 回到初始状态
		zap.setPosition(2, 0.0);
		zap.setPosition(1, 0.2);
		zap.setPosition(0, start_pos);
		training_curve_rdata.shrink(0);
		training_curve_rdata.push_back(ImVec2(0.0f,0.0f));
		meas_cnt = 0;
	} ImGui::SameLine();

	if(ImGui::Button(reader_running ? "stop " : "start")) {
		reader_running = !reader_running;
	
	}

	if (reader_running) {
		// 图像采集
		/// \~chinese 开始图像采集				        \~english Start grabbing images
		StartGrabImage(&cam);
		/// \~chinese 创建工作线程用于获取图像              \~english Create a worker thread that is used to acquire the images
		unsigned int nThreadID = 0;
		void* hThreadHandle = (void*)_beginthreadex(NULL, 0, WorkThread, &cam, 0, &nThreadID);

		if (NULL == hThreadHandle)
		{
			printf("fail to create workThread to grab image.\n");
			ItkManTerminate();
			pressEnterToExit();
			exit(EXIT_FAILURE);
		}

		/// \~chinese 等待图像采集结束				    \~english Wait for grabbing images finished
		//{ int getCharTmp = getchar(); }
		/// \~chinese 停止工作线程                    \~english stop workThread
		WaitForSingleObject(hThreadHandle, INFINITE);
		CloseHandle(hThreadHandle);
		//printf("current image %d, pos = %3.3fmm, I = %f\n", meas_cnt, zap.m_position[0], static_cast<float>(sum_pixels >> 16));
		if (0 == meas_cnt) {
			training_curve_rdata.shrink(0);
		}
		training_curve_rdata.push_back( ImVec2(zap.m_position[0], static_cast<float>(sum_pixels>>16) ));
		//printf("rdata buffer size = %d\n", training_curve_rdata.Size);
		g_bExit = false;
		sum_pixels = 0;
		meas_cnt++;
		/// \~chinese 停止图像采集				        \~english Stop grabbing images
		ret = IKapStopGrab(cam.g_hBoard);
		CHECK_IKAPBOARD(ret);
		
		if (zap.m_position[0] < end_pos) {
			zap.m_position[0] += move_step;
			zap.setPosition(0, zap.m_position[0]);
			Sleep(100);
		}
		else {
			reader_running = false;
		}
	}
	float min_y = 100.0, max_y = 0.0;
	if (training_curve_rdata.Size > 2) {
		get_rdata_ylim(training_curve_rdata, min_y, max_y);
		//printf("min_y = %f, max_y = %f\n", min_y, max_y);
	}
	
	if (ImPlot::BeginPlot("##Rolling", ImVec2(-1, 400))) {
		ImPlot::SetupAxes("position[mm]", "energy");
		ImPlot::SetupAxesLimits(min(start_pos, end_pos), max(start_pos, end_pos), min_y, max_y, ImGuiCond_Always);
		ImPlot::PlotLine("energy vs Position", &training_curve_rdata[0].x, &training_curve_rdata[0].y, training_curve_rdata.size(), 0, 0, 2 * sizeof(float));
		ImPlot::EndPlot();
	}



	ImGui::End();
}


std::vector<std::string> EnumerateAvailableSerialPorts() {
	std::vector<std::string> availablePorts;

	// Windows串口编号通常从1到255，枚举合理范围（1-20即可覆盖大部分场景）
	for (int portNum = 1; portNum <= 20; ++portNum) {
		// 拼接串口名，如"COM1"、"COM3"
		std::string portName = "\\\\.\\COM" + std::to_string(portNum);

		// 尝试以只读方式打开串口，不独占（避免占用正在使用的串口）
		HANDLE hSerial = CreateFileA(
			portName.c_str(),          // 串口名
			GENERIC_READ,              // 只读权限
			FILE_SHARE_READ | FILE_SHARE_WRITE, // 共享读写
			NULL,                      // 安全属性
			OPEN_EXISTING,             // 仅打开已存在的串口
			0,                         // 无特殊属性
			NULL                       // 模板文件
			);

		// 打开成功表示串口存在且可用
		if (hSerial != INVALID_HANDLE_VALUE) {
			availablePorts.push_back("COM" + std::to_string(portNum));
			CloseHandle(hSerial); // 关闭句柄，不占用串口
		}
	}

	return availablePorts;
}

std::wstring to_wstring(const std::string& str) {
	int len = MultiByteToWideChar(CP_ACP, 0, str.c_str(), (int)str.size(), NULL, 0);
	std::wstring wstr(len, 0);
	MultiByteToWideChar(CP_ACP, 0, str.c_str(), (int)str.size(), &wstr[0], len);
	return wstr;
}


void get_rdata_ylim(ImVector<ImVec2> rdata, float &min_y, float& max_y) {
	min_y = rdata[0].y;
	max_y = rdata[0].y;
	for (int i = 1; i < rdata.Size; i++) {
		if (rdata[i].y > max_y) {
			max_y = rdata[i].y;
		}

		if (rdata[i].y < min_y) {
			min_y = rdata[i].y;
		}
	}
}