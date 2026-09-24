# FF C++ SDK 发布说明

## 1.1.0（当前）

发布日期：2026-09-01。此版本与 FF GUI 1.1.0 对应，公共 API 为 C++17，目标平台为 Windows x64 / Visual Studio 2019 v142。

### 新增与完成

- 发布 `FF_SDK/` 交付目录：包含公共头文件、Release 静态库、MightyZap 运行时依赖、配置例子/模板及接口文档。
- 加入 `Training.hpp` 公共接口：能量累加、仿真扫描、谷值检测、Layer 2 杆位一对一分配、拖拽可行区间与 JSON 会话存取。
- 加入 `FFApplicationService`：封装配置装载、仿真/真实后端选择、连接、目标校验、计划预览/执行、取消和急停，供 GUI 或上层应用使用。
- 配置固定为 schema v2：三 Channel、逐层 `layer_members`、`clearance_rules`、`training_rod_id` 和线性 P0/P1 光学标定。
- `submitMotion()` 在下发前刷新状态并统一检查地址、限位、禁区、层拓扑和相邻杆间隙；整个请求验证失败时不会发出目标命令。

### 兼容性和升级

- 1.1.0 新增 Training 公共头及其符号；使用旧 1.0 静态库的应用必须重新链接 `ff_core.lib`。
- 旧 schema v1 JSON 可以读取为兼容输入；新配置应保存为 schema v2。真实运行所需安全字段缺失时，不能将该配置视为已验证的机械配置。
- `training_rod_id` 是 Channel 拓扑配置，必须是同 Channel 的 Layer 2 Rod；未配置时写 JSON `null` 或将 CSV 字段留空。
- 厂商 DLL/导入库仍为 `CppDllMightyZap_Ver2010`。其许可和部署条件由厂商决定，不随 FF SDK 授予。

### 已知实现边界（必须阅读）

- `IControllerBackend::setGoalsSynchronous()` 是抽象接口；当前 MightyZap 适配实现按杆调用厂商 `GoalPosition`，因此跨杆/跨串口仅是软件级近同步，**不能**作为硬实时总线同步承诺。
- `MotionPlanner::plan()` 生成安全位与目标位的顺序步骤；`FFApplicationService::executePlan()` 会逐步等待位置容差与稳定时间。机械轨迹、加速度包络和同步转换的工程验证仍由集成方/安全评审负责。
- `setExternalSafetyOverride()` 当前为兼容占位接口，不改变安全判定，不能用于绕过通信、报警、限位或拓扑检查。
- Simulation 后端会立即更新位置，适合接口和规则测试；它不模拟真实通讯延迟、机械动力学、碰撞或硬件互锁。

### 发布校验

发布前运行：

```powershell
msbuild FF.sln /p:Configuration=Release /p:Platform=x64
.\x64\Release\ff_tests.exe
powershell -ExecutionPolicy Bypass -File tools/package_ff_sdk.ps1 -Configuration Release -SkipBuild
```
