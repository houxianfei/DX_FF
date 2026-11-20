# FF 应用层 SDK 1.0.1 包说明（未正式发布）

正式交付产物为 `publish/sdk/FF_SDK_v1.0.1.zip`，解压得到同名目录 `FF_SDK_v1.0.1/`。目标环境为 Windows x64、Visual Studio 2019 v142、C++17。`FF_SDK/` 是供 GUI 编译使用的本地暂存目录，对外交付使用版本化 ZIP。

## 发布命令与版本

在源码仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_sdk.ps1 -Configuration Release
```

版本唯一来源为内部头文件 `include/ff/Version.hpp` 的 `FF_SDK_VERSION`，同时用于 SDK 的 `version()`、包内 VERSION、ZIP 名称及解压目录。升级时修改该版本值并执行完整发布命令，勿在修改版本或源码后用 SkipBuild 复用旧 DLL；同步更新接口文档与版本记录。内部版本头文件不交付，公开 include 始终只有两个文件。

不同版本生成不同 ZIP，旧版本保留；同版本重新发布会在新 ZIP 完整生成后替换原文件。Debug 只生成本地暂存目录，不覆盖 Release ZIP。`-SkipArchive` 仅更新本地 SDK，普通 GUI 构建使用此选项。

## 压缩包内容

以下路径均相对于解压后的版本目录。

| 路径 | 内容 |
| --- | --- |
| `include/DX_FF.h` | `FF::ff_driver` 对象接口；集成方的包含入口 |
| `include/DX_FF_Types.h` | 结果码、状态和数据结构；由 `DX_FF.h` 自动包含 |
| `lib/x64/Release/ff_application.lib` | DLL 导入库 |
| `bin/x64/Release/ff_application.dll` | SDK 运行时 |
| `bin/x64/Release/CppDllMightyZap_Ver2010.dll` | 厂商运行时依赖 |
| `config/` | 机台 JSON 示例及 CSV 编辑模板 |
| `docs/` | 场景指南、完整 Training、接口/类型/文件格式、错误码、安全部署、升级说明、仿真步骤及 GUI 操作说明 |
| `examples/` | C++ 独立示例及配套仿真配置和 slot0 |
| `tools/ff_config_convert.py` | 配置校验及 JSON/CSV 转换工具，仅依赖 Python 标准库 |
| `runtime/vc_redist.x64.exe` | Microsoft Visual C++ x64 运行库安装程序（Release ZIP） |
| `README.md`、`PACKAGE_README.md`、`RELEASE_NOTES.md`、`VERSION` | 快速开始、包说明、版本记录与版本号 |

## 解压后的使用步骤

1. 解压到任意本地目录；先阅读 README，按业务进入场景指南或 Training 文档。
2. 目标机尚无兼容 Visual C++ x64 运行库时，运行 runtime/vc_redist.x64.exe 完成安装；已安装兼容运行库可直接使用。
3. 将 `include` 加入包含目录，链接同一版本的 `ff_application.lib`，并部署对应的 DLL。应用代码只需 `#include <DX_FF.h>`。
4. 先用包内示例验证 Simulation；配套配置不可直接用于真实机台。编译命令见[仿真示例](public_sdk/仿真示例.md)。

配置工具示例（从解压目录执行）：

```powershell
python tools/ff_config_convert.py validate config/ff_config.example.json
python tools/ff_config_convert.py csv-to-json config my_ff_config.json
```

硬件集成前应替换并复核机台配置、串口和 slot0。

使用顺序：连接 `open()` → `initialize(slot0)` → 业务操作 → `close()`。单杆调试及建立初始位置可在初始化前执行。训练包含训练杆到位、外部 stage/TDI 同区域采集、能量计算、寻谷、分配（含避让）、完整目标执行及效果验证。FF SDK 不控制 stage/TDI；当前 GUI Real 采集未接入。

按业务阅读源码的[场景指南](public_sdk/应用场景指南.md)、[Training 完整流程](public_sdk/Training计算流程.md)和[仿真示例](public_sdk/仿真示例.md)。发布后入口为 SDK README，文档位于 docs，代码及仿真配置位于 examples。

新位置文件 FF-POS-2 仅列记录通道，普通配方可缺整个通道；用 send_partial_rods 下发并轮询。move_rods/send_rods 要求完整启用杆集合。初始化 slot0 必须完整。多通道会话新写 FF-TRAIN-3，旧格式继续可读，加载后应重新规划。
