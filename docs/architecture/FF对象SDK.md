# FF 整机对象 SDK

本文档对应内部实现头文件 `include/ff/FF.hpp` 和 `include/ff/Training.hpp`，描述 FF 的对象模型、配置、安全边界、应用服务和 Training 能力。这些头文件不随应用层 SDK 1.0.1 交付。每个 Channel 使用一个独立控制器和一个串口；每个控制器可管理数量不同的 MightyZap 推杆。

**适用平台与边界：** SDK 的发布目标是 Windows x64、Visual Studio 2019 v142、C++17。Simulation 后端用于功能测试，不等同于机械仿真或安全认证；真实运行前必须完成独立的硬件安全设计、限位验证和现场联调。

```text
FF
├── CH1 ── Controller ── COMx ── Rod 0..N
├── CH2 ── Controller ── COMy ── Rod 0..M
└── CH3 ── Controller ── COMz ── Rod 0..K
```

## 1. 标识规则

`RodId` 是控制器内的局部 MightyZap ID，类型为 `uint8_t`。不同控制器下可以出现相同杆号，例如 `CH1/Rod0` 与 `CH2/Rod0`。整机范围内必须使用复合地址：

```cpp
ff::RodAddress address{ff::ChannelId::CH1, 0};
```

`RodAddress` 支持比较，`RodAddressHash` 可用于 `unordered_map`。

## 2. 配置

`FFConfig` 包含机台编号、配置版本、三个独立的 `FFChannelConfig`、活动杆数量上限和状态过期时间。

每个 `FFChannelConfig` 包含 `ChannelId`、一个 `ControllerConfig`、可选的 `training_rod_id` 和该控制器下的 `RodConfig` 列表。`training_rod_id` 是机台拓扑配置：设置时必须指向本 Channel 中存在的 Layer 2 Rod；未配置时写 `null`。Training UI 只读取此字段，不允许用 INI 或界面控件替换它。初始化应检查：三个 Channel 齐全且不重复、串口名不重复、同一 Channel 内杆 ID 不重复、ID 在设备允许范围内、软限位位于硬限位内。

`RodConfig` 保存杆的限位、零位、安全位、位置换算、运动方向、速度、加减速度、到位容差、稳定时间、超时时间和层掩码。

## 3. 配置文件读写

磁盘上的规范格式为 UTF-8 JSON。`FFConfigIO::load()` 将 JSON 解析为 `FFConfig`；`FF` 再按三个 Channel 配置创建 `FFChannel`、`Controller` 和 Rod 容器。

```cpp
auto loaded = ff::FF::loadConfig("ff_config.json");
if (!loaded) return;

ff::FF machine(loaded.value, std::move(backends));
machine.initialize();
machine.saveConfig("ff_config_copy.json");
```

`applyConfig()` 只允许在 Offline 状态调用。`saveConfig()` 保存静态配置和标定参数，不保存实时位置、运动状态或报警；运行状态属于 `FFStatus`。

JSON 示例：

```json
{
  "schema": "ff-config",
  "schema_version": 2,
  "machine_id": "FF-001",
  "config_version": "FF-CONFIG-0.1",
  "max_active_rod_count": 8,
  "status_expiration_ms": 200,
  "channels": [
    {
      "id": "CH1",
      "controller": {
        "name": "CTRL-CH1",
        "serial": { "port": "COM3", "baud_rate": 115200, "timeout_ms": 500 },
        "poll_period_ms": 20,
        "retry_count": 3
      },
      "training_rod_id": 0,
      "rods": [
        {
          "id": 0,
          "name": "CH1-R0",
          "hard_min": 0,
          "hard_max": 4095,
          "soft_min": 100,
          "soft_max": 4000,
          "zero_position": 2048,
          "safe_position": 3900,
          "safe_position_valid": true,
          "P0": 0.0,
          "P1": 0.021978021978,
          "inward_direction": 1,
          "max_velocity": 300.0,
          "max_acceleration": 100.0,
          "max_deceleration": 100.0,
          "position_tolerance": 5,
          "stable_time_ms": 100,
          "move_timeout_ms": 5000
        }
      ],
      "layer_members": [
        { "rod_id": 0, "layer": 1, "role": 3, "left_neighbor": -1, "right_neighbor": -1 }
      ]
    },
    { "id": "CH2", "controller": { "name": "CTRL-CH2", "serial": { "port": "COM4", "baud_rate": 115200, "timeout_ms": 500 } }, "training_rod_id": null, "rods": [] },
    { "id": "CH3", "controller": { "name": "CTRL-CH3", "serial": { "port": "COM5", "baud_rate": 115200, "timeout_ms": 500 } }, "training_rod_id": null, "rods": [] }
  ]
}
```

字段采用 `snake_case`；Channel ID 使用 `CH1`、`CH2`、`CH3`，Rod ID 是控制器内整数地址。不同 Channel 可以重复使用相同 Rod ID。

层拓扑采用 `layer_members`。每条记录只表示一根 Rod 在一个普通整数 layer 中的成员关系；同一 Rod 跨层时写多条记录，并为每条记录单独提供邻居关系。

## 4. SDK 后端适配

公开头文件不直接包含 `CppDllMightyZap_Ver2010.h`。实现层通过 `IControllerBackend` 适配 MightyZap SDK：

| 后端接口 | MightyZap 能力 |
|---|---|
| `open` / `close` | `OpenMightyZap` / `CloseMightyZap` |
| `setGoal` | `GoalPosition` 及速度参数 |
| `readStatus` | `PresentPosition`、`ReadError` |
| `enable` | `ForceEnable` |
| `stop` / `stopAll` | `ForceEnable(id, false)`；仅作用于适配器已接触过的 ID |
| `home` | 项目定义的回零流程 |
| `resetAlarm` | 错误复位流程 |

一个 `Controller` 独占一个后端和一个串口。同一控制器的 SDK 调用由 `ioMutex_` 串行化；不同控制器可以独立操作。由于原 SDK 部分函数返回 `void`，调用方必须结合 `refreshStatus()`、位置容差、稳定时间和超时处理执行结果。

### 当前同步语义

`IControllerBackend::setGoalsSynchronous()` 表示“作为一组目标提交”的扩展点。当前 Simulation 与 MightyZap 适配实现均逐杆调用 `setGoal()` / `GoalPosition()`；因此它不能提供厂商总线同步写或硬实时同步保证。`MotionRequest::allowSoftwareSynchronousStart` 也不把不同串口的命令升级为硬实时同步。若工程需要同步启动，集成方必须在后端中实现并验证厂商同步写机制，并保持同一安全校验语义。

## 5. 生命周期

```cpp
ff::FF machine(config, std::move(backends));
auto result = machine.initialize();
if (!result) return;

result = machine.connect();
if (!result) return;

machine.refreshStatus();
```

结束运行：

```cpp
machine.stopAll();
machine.disconnect();
```

`FF`、`FFChannel` 和 `Controller` 禁止复制，以保证串口资源唯一。

## 6. 运动控制

运动接口采用“非阻塞提交 + 状态查询”。成功只表示命令通过校验并提交，不表示推杆已到位。

```cpp
ff::MotionRequest request;
request.allowSoftwareSynchronousStart = true;
request.targets = {
    {{ff::ChannelId::CH1, 0}, 1200, 300.0},
    {{ff::ChannelId::CH2, 0},  900, 250.0},
    {{ff::ChannelId::CH3, 2}, 1500, 280.0}
};

auto receipt = machine.submitMotion(request);
if (!receipt) {
    // receipt.error / receipt.message
}
```

`submitMotion` 的处理顺序是：检查连接状态、校验复合地址、检查重复目标、检查软限位和安全规则、按 Channel 分组、分别提交到三个控制器。`refreshStatus()` 与 `status()` 用于观察位置、运动状态、报警和反馈时间。

不同串口之间只能提供软件级近同步，不能视为硬实时同步。

### 通道内全部 Rod 同步回安全位

对于“当前通道已经安全，所有杆同时退回各自安全位”的场景，使用：

```cpp
auto result = channel->moveAllToSafePositions();
```

也可以从整机对象按 Channel 调用：

```cpp
auto result = machine.moveChannelToSafePositions(ff::ChannelId::CH1);
```

如果安全位置由上层规划器或用户输入，则使用显式目标接口：

```cpp
std::vector<ff::RodTarget> targets = {
    {{ff::ChannelId::CH1, 0}, 1200, 300.0},
    {{ff::ChannelId::CH1, 1},  950, 300.0},
    {{ff::ChannelId::CH1, 2}, 1100, 300.0}
};

auto result = channel->moveAllToPositions(targets);
// 或：machine.moveChannelToPositions(ff::ChannelId::CH1, targets);
```

`moveAllToPositions()` 的位置输入就是 `targets`。函数会先检查当前 Channel 安全，再检查每个目标的 Channel、地址和单杆合法位置，随后调用 `setGoalsSynchronous()` 作为一组提交。它不要求目标覆盖全部 Rod，也不在该接口中重复整机层拓扑/间隙验证；需要完整规则验证时，优先调用 `FF::submitMotion()` 或 `FFApplicationService::setRodTargets()`。当前后端的“同步”实现为顺序下发，详见第 4 节。

`moveAllToSafePositions()` 只是使用每根 `RodConfig::safePosition` 自动生成上述 `targets` 的便捷方法。

该函数必须执行以下流程：

1. 使用当前 Rod 状态和位置确认 Channel 为 `SafetyState::Safe`；
2. 检查每根 Rod 都有 `safePositionValid == true`，且安全位位于软限位内；
3. 为所有 Rod 构造目标列表；
4. 通过 `IControllerBackend::setGoalsSynchronous()` 作为一组下发；
5. 返回后由 `refreshStatus()` 查询是否全部到位。

如果任一杆状态未知、报警、没有安全位或目标非法，则整个操作拒绝下发。状态过期检查目前不是 `Controller::evaluateSafety()` 的实现条件；需要新鲜反馈时，调用方应先 `refreshStatus()` 或使用会自动刷新状态的 `FF::submitMotion()`。当前后端逐杆下发，因此不承诺同时启动。

## 7. 状态与错误

`FF::status()` 返回整机 `FFStatus`，包括整机状态、每个 Channel 的连接状态/最后错误以及每根杆的 `RodStatus`。

常见 `ErrorCode`：`InvalidConfiguration`、`DuplicateId`、`DuplicatePort`、`RodNotFound`、`LimitViolation`、`SafetyViolation`、`Communication`、`Timeout`、`DeviceAlarm` 和 `EmergencyStopped`。

## 8. Channel 安全状态

MightyZap SDK 不直接提供 Channel 安全结论，只提供当前位置、限位和设备报警等原始数据。`FFChannel` 在这些数据之上执行安全判断：

```cpp
// 只使用最近一次 refreshStatus() 缓存的数据，不产生串口 I/O
auto current = channel->controller().status();
auto safety = channel->evaluateSafety();

// 读取 SDK 状态后立即判断
auto fresh = channel->refreshAndEvaluateSafety();
if (fresh && fresh.value.state == ff::SafetyState::Safe) {
    // 允许继续规划或运动
}
```

`ChannelSafetyStatus` 的 `state` 取值为 `Unknown`、`Safe`、`Unsafe`、`Checking` 或 `Fault`；`violations` 可包含 `PositionUnknown`、`DeviceAlarm`、`SoftLimitViolation`、`CommunicationLost`、`RodInterference` 与 `InvalidTopology` 等。

`Controller::evaluateSafety()` 基于缓存状态检查：位置是否有效、设备是否报警、当前位置是否符合单杆硬/软限位，以及控制器是否连接。`FF::status()` 在此基础上额外执行层拓扑/间隙校验，并用 `RodInterference` 或 `InvalidTopology` 标记不安全。禁止区和完整目标组合由 `FF::validateMotion()` 在命令验证时检查。`FF::refreshChannelSafety()` 和 `FF::channelSafety()` 提供整机按 Channel 查询入口。

`setExternalSafetyOverride()` 目前是为 API 兼容保留的无操作接口：它返回成功但不改变安全结论。不得将其作为绕过任何安全检查的途径。

## 9. 停止与急停

```cpp
machine.stopAll();
machine.emergencyStop();
machine.resetFaults();
```

软件急停不能替代独立硬件安全回路。即使一个控制器通信失败，也应继续尝试停止其他控制器，并记录失败通道。

## 10. 测试重点

- `CH1/Rod0` 与 `CH2/Rod0` 能路由到不同串口；
- 同一 Channel 内重复杆 ID、重复串口、缺失 Channel 会被拒绝；
- 跨三个 Channel 的批量目标正确分组；
- 越过软限位、不存在的复合地址、重复目标不会下发；
- 单个串口掉线不影响其他 Channel 的状态读取；
- 同一控制器并发调用不会交叉写串口；
- `stopAll` 和 `emergencyStop` 会尝试操作全部控制器。

## 11. 上层应用服务

`FFApplicationService` 是 GUI 或其他上层程序的推荐入口。它持有一个运行时 `FF`，并负责在加载/应用配置时注入 Simulation 或 MightyZap 后端、覆盖运行时串口和限制活动 Channel 数。其方法内部带锁；应用层不应同时绕过它直接驱动同一台 `FF`。

典型流程：

```cpp
ff::FFApplicationService service;
auto loaded = service.loadConfig("ff_config.json", true); // true = Simulation
if (!loaded || !service.connect()) return;

const ff::RodTarget target{{ff::ChannelId::CH1, 0}, 1200, 20.0};
auto checked = service.validateRodTargets({target}); // 只验证，不下发
if (checked) service.setRodTarget(target);           // 验证并下发
```

- `loadConfig()` / `applyConfig()`：只能在断开后调用；会先执行配置校验。
- `setRodTarget()` / `setRodTargets()`：通过 `FF::submitMotion()`；该路径会刷新所有 Channel 状态，并对完整请求执行地址、限位、禁区、拓扑和间隙校验。
- `validateRodTargets()`：刷新后只校验，不发送命令。
- `previewPlan()`：根据当前快照生成计划，不连接也可调用；计划必须在同一 `configVersion` 下执行。
- `executePlan()`：逐步下发计划并等待每根 Rod 达到配置的 `positionTolerance`、持续 `stableTime`；超时、报警或配置版本变化即失败，不自动恢复。
- `cancelMotion()` 与 `emergencyStop()`：调用整机停止；软件停止不能替代独立硬件回路。

## 12. Training 公共接口

Training API 位于 `ff/Training.hpp`，与 GUI 解耦。`TrainingSession` 使用 schema `FF-TRAIN-1` 保存扫描设置、样本、谷值候选、分配结果、保存瞬间的 Rod 状态和可执行目标集；会话必须携带 `machineId` 与 `configVersion`，上层在执行前应核对二者。

`TrainingAnalysis` 提供灰度能量累加、仿真扫描能量、谷值检测、Layer 2 杆位的一对一最小行程分配、拖拽可行区间，以及会话 JSON 的读写。`training_rod_id` 始终由机台配置指定，必须是同 Channel 的 Layer 2 Rod。`ItekEnergySource` 是硬件采集扩展点；当前发行版不捆绑经验证的 Itek DLL 或相机配置，因此真实采集不可用，不能把 Simulation 扫描结果用于光学或机械验收。

## 13. 文件关系

```text
include/ff/FF.hpp                      公共对象模型和接口
FF.cpp                                  FF/Channel/Controller/Rod 实现
MightyZapControllerBackend.cpp          MightyZap SDK 适配层
CppDllMightyZap_Ver2010.h/.lib/.dll     厂商 SDK
```

## 14. CSV/JSON 转换工具

转换脚本位于 `tools/ff_config_convert.py`，仅使用 Python 标准库，不依赖第三方 JSON/CSV 包。

```text
python tools/ff_config_convert.py csv-to-json config/templates config.json
python tools/ff_config_convert.py json-to-csv config.json config_csv_export
python tools/ff_config_convert.py validate config/templates
python tools/ff_config_convert.py validate config.json
```

CSV 配置目录至少包含 `system.csv`、`channels.csv` 和 `rods.csv`。`channels.csv` 的 `training_rod_id` 为可选列值：填写时必须是该 Channel 的 Layer 2 Rod；留空会生成 JSON `null`。脚本读取 CSV 后先执行结构和数值校验，校验失败时不会生成 JSON；JSON 导出前也会执行同样的校验。推荐将 CSV 目录作为人工审核源，将生成的 JSON 作为 FF 程序运行文件。
