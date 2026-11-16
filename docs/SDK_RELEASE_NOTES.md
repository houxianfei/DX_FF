# FF 应用层 SDK 1.0.1 版本记录

**状态：未正式发布。** FF GUI 与应用层 SDK 的当前本地版本均为 `1.0.1`。此前开发构建使用过的版本号不作为正式发布历史，也不提供跨开发构建的 ABI 兼容承诺。

## 交付内容

- `DX_FF.h` 对象接口、`DX_FF_Types.h` 数据结构与 `DX_FF_MotionPolicy.h` 固定运动速度宏。
- Windows x64 / Visual Studio 2019 v142 / C++17 的 `ff_application.lib` 和 `ff_application.dll`。
- MightyZap 运行时 DLL、配置样例、接口文档与独立 SDK 示例。

本地开发构建的运动接口已移除逐次输入的速度参数，目标及位置文件也不再写速度。`FF_MOTION_SPEED` 默认为底层原始速度最大值 `1023`，MightyZap 在连接初始化时设置一次；旧位置文件的速度字段读取时忽略。此变更需重新编译 SDK 消费方。

公开运动入口收敛为阻塞的 `move_rod()`、`move_rods()` 和非阻塞的 `send_rod()`、`send_rods()`。多杆方法直接接收每根启用杆恰好一个目标，安全校验在 SDK 内进行。`load_positions()` 改为返回 `ValueResult<PositionSet>`，其中包含目标数组和摘要，不保留待执行目标。公开的初始化、目标缓存设置与校验、规划及旧执行方法和计划类型已移除；已有 SDK 消费方须按新头文件修改调用并重新编译。

位置文件现用独立的 `FF-POS-1` 格式，固定列出三个通道，各杆目标分行；不再写入 Training session 的扫描字段。新增 `save_positions()` 用于保存指定完整目标集，GUI 预设也通过该入口写入。旧 `FF-TRAIN-1` 位置文件仍可读取。

Training 新增三通道归档 `FF-TRAIN-2` 和公开的保存、读取接口。GUI 会一起保存三个通道的训练缓存；数组逐项分行，浮点字段最多三位小数。旧单通道接口仍可使用，旧文件仍可读取；新增的公开类型与接口要求 SDK 调用方重新编译。

## 发布前验证

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_sdk.ps1 -Configuration Release
.\x64\Release\ff_tests.exe
```

真实硬件上线前仍需独立完成机械限位、速度、互锁和安全边界验证；Simulation 只验证软件流程。
