# FF 工程

FF 是一套包含 3 个 Channel 和多根 MightyZap 遮挡棒的整机控制、训练与运动规划工程。当前本地版本为 **FF GUI 1.0.1 / FF 应用层 SDK 1.0.1**，尚未正式发布。GUI 面向操作人员；SDK 面向集成方，二者共享同一配置、安全校验和后端适配层。

## 目录说明

| 目录 | 用途 |
| --- | --- |
| `include/DX_FF.h` | SDK 对象入口头文件；自动包含 `DX_FF_Types.h` |
| `include/DX_FF_Types.h` | SDK 公开数据结构、结果码与状态定义 |
| `include/ff/` | SDK DLL 内部实现头文件，不随 SDK 交付 |
| `app/` | GUI 入口，以及按设置、仪表盘、训练和参数编辑拆分的界面实现 |
| `src/core/` | 核心控制、训练、配置与 SDK 适配实现 |
| `third_party/imgui/` | GUI 所用的 ImGui、ImPlot 和文件对话框源码 |
| `third_party/MightyZap/` | MightyZap 厂商 SDK 的头文件、导入库和运行时 DLL |
| `FF_SDK/`、`publish/` | 发布脚本生成的 SDK、程序包和安装包；不作为源码维护 |
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

工程由 `ff_core`、`ff_gui`、`ff_tests` 和 `ff_application` 四个 VS2019/v142 x64 项目组成。历史 demo 已清理；GUI 第三方源码位于 `third_party/imgui/`。

Debug 构建并运行测试：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_gui_app.ps1 -Configuration Debug
.\x64\Debug\ff_tests.exe
```

生成 Release 程序目录及 Inno Setup 安装包：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_gui_app.ps1 -Configuration Release
powershell -ExecutionPolicy Bypass -File tools/build_ff_gui_installer.ps1 -Configuration Release -SkipPackage
```

安装包输出为 `publish/installer/FF_GUI_Setup_v1.0.1.exe`。程序默认使用 Simulation 模式；配置、日志和运行数据位于 `%PROGRAMDATA%\RSIC\FF`。

## SDK 发布

SDK 发布包面向 Windows x64 / VS2019 v142 / C++17 集成。它包含对象入口 `DX_FF.h` 和数据结构 `DX_FF_Types.h`，以及 DLL、导入库、MightyZap 运行时依赖、配置样例和中文接口文档。集成方仍只需包含 `<DX_FF.h>`。GUI 构建会先生成匹配配置的 SDK。单独构建 SDK：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_sdk.ps1 -Configuration Release
```

输出目录为 `FF_SDK/`；入口文档是 `FF_SDK/README.md`，其余应用层文档见 `FF_SDK/docs/`。SDK 版本与 GUI 版本独立标识。

> 真实硬件上线前，必须在目标机完成限位、零位、标定、拓扑、禁区和间隙规则的复核。Simulation 仅用于软件验证，不能证明机械安全性。

## 后续开发入口

- 对外接口：`include/DX_FF.h`（`namespace FF`）
- GUI 私有配置编辑接口：`app/private_config.hpp`、`app/private_config.cpp`（不随 SDK 交付）
- SDK 依赖：`third_party/MightyZap/`
- 对象模型说明：`docs/architecture/FF对象SDK.md`
- 运动规划说明：`docs/design/遮挡棒坐标与运动规划设计.md`

本地构建产物位于 `x64/`；发布脚本可重新生成 `FF_SDK/` 和 `publish/`。
