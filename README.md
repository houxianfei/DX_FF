# FF 工程

FF 是一套包含 3 个 Channel 和多根 MightyZap 遮挡棒的整机控制、训练与运动规划工程。当前发布为 **FF GUI 1.1.0 / FF C++ SDK 1.1.0**。GUI 面向操作人员；SDK 面向集成方，二者共享同一配置、安全校验和后端适配层。

## 目录说明

| 目录 | 用途 |
| --- | --- |
| `include/ff/` | FF 对外公共 C++ 接口 |
| `FF_SDK/` | 可直接交付的 SDK 发布包（由发布脚本生成） |
| `third_party/MightyZap/` | MightyZap 厂商 SDK 的头文件、导入库和运行时 DLL |
| `docs/architecture/` | SDK、对象模型和生命周期说明 |
| `docs/design/` | 坐标体系、拓扑、安全和运动规划设计 |
| `config/templates/` | 可编辑的 CSV 配置模板 |
| `tools/` | 配置转换与校验工具 |
| `generated/` | Excel、检查报告等生成文件 |
| `.codex_artifacts/` | 工具运行产生的辅助资料和缓存 |

## 配置转换工具

工具只依赖 Python 标准库。命令请从工程根目录执行：

```powershell
python tools/ff_config_convert.py csv-to-json config/templates ff_config.json
python tools/ff_config_convert.py json-to-csv ff_config.json config/exported
python tools/ff_config_convert.py validate config/templates
python tools/ff_config_convert.py validate ff_config.json
```

规范配置格式为 UTF-8 JSON；CSV 主要用于人工编辑和交换。训练杆由每个 Channel 的 `training_rod_id` 配置：JSON 使用 Rod ID 或 `null`，CSV 使用 `channels.csv` 的同名列（留空表示未配置）；填写时必须为该 Channel 的 Layer 2 Rod。

## 正式版工程

正式版由 `ff_core`、`ff_gui` 和 `ff_tests` 三个 VS2019/v142 x64 项目组成。`sample/FFdemo1` 是历史 demo，仅供行为参考。

Debug 构建并运行测试：

```powershell
msbuild FF.sln /p:Configuration=Debug /p:Platform=x64
.\x64\Debug\ff_tests.exe
```

生成 Release 程序目录及 Inno Setup 安装包：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_gui_app.ps1 -Configuration Release
powershell -ExecutionPolicy Bypass -File tools/build_ff_gui_installer.ps1 -Configuration Release -SkipPackage
```

安装包输出为 `publish/installer/FF_GUI_Setup_v1.1.0.exe`。程序默认使用 Simulation 模式；配置、日志和运行数据位于 `%PROGRAMDATA%\RSIC\FF`。

## SDK 发布

SDK 发布包面向 Windows x64 / VS2019 v142 / C++17 集成。它包含公共头文件、Release 静态库、MightyZap 运行时依赖、配置样例和详细接口文档。构建并生成发布包：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_sdk.ps1 -Configuration Release
```

输出目录为 `FF_SDK/`；入口文档是 `FF_SDK/README.md`，完整接口、安全模型与训练会话格式见 `FF_SDK/docs/FF_SDK_Reference.md`。SDK 版本与 GUI 版本独立标识，但本次二者均为 1.1.0。

> 真实硬件上线前，必须在目标机完成限位、零位、标定、拓扑、禁区和间隙规则的复核。Simulation 仅用于软件验证，不能证明机械安全性。

## 后续开发入口

- 公共接口：`include/ff/FF.hpp`
- SDK 依赖：`third_party/MightyZap/`
- 对象模型说明：`docs/architecture/FF对象SDK.md`
- 运动规划说明：`docs/design/遮挡棒坐标与运动规划设计.md`

后续可在此基础上补充 `src/`、`tests/` 和 CMake 工程文件。
