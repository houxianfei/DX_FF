# FF 应用层 SDK 1.0.1 包说明（未正式发布）

`tools/package_ff_sdk.ps1` 生成 `FF_SDK/` 本地交付目录。目标环境为 Windows x64、Visual Studio 2019 v142、C++17。

| 路径 | 内容 |
| --- | --- |
| `include/DX_FF.h` | `FF::ff_driver` 对象接口；集成方的包含入口 |
| `include/DX_FF_Types.h` | 结果码、状态和数据结构；由 `DX_FF.h` 自动包含 |
| `include/DX_FF_MotionPolicy.h` | 旧版兼容宏 `FF_MOTION_SPEED`；当前设备速度由配置的 `motor_parameters.speed_limit_raw` 设置 |
| `lib/x64/Release/ff_application.lib` | DLL 导入库 |
| `bin/x64/Release/ff_application.dll` | SDK 运行时 |
| `bin/x64/Release/CppDllMightyZap_Ver2010.dll` | 厂商运行时依赖 |
| `config/`、`docs/`、`examples/` | 配置样例、接口文档和独立示例 |

集成时将 `include` 加入包含目录，链接同一版本的 `ff_application.lib`，并部署对应的 DLL。应用代码只需 `#include <DX_FF.h>`。详细用法见生成包中的 `README.md`。

调用顺序：`open()` → `move_rod()` → `close()`；位置文件为 `open()` → `load_positions()` → `move_rods(loaded.value.targets)` → `close()`；训练为扫描取样 → `find_valleys()` → `assign_training()` → 合成完整目标集 → `move_rods()` → `record_positions()`。阻塞 `move_*` 等到稳定到位，非阻塞 `send_*` 仅表示目标下发成功。完整示例见生成包的 `docs/快速开始.md` 与 `docs/Training计算流程.md`。

位置文件采用独立的 `FF-POS-1` 格式。`save_positions()` 写指定目标集，`record_positions()` 写当前反馈；两者均按三个通道分组并为每根杆单独写一行。
