# FF GUI 1.0.1 操作说明（未正式发布）

FF GUI 1.0.1 默认使用 **Simulation** 模式。它与 FF 应用层 SDK 1.0.1 共用 JSON/CSV 配置、目标校验和安全规则；Simulation 用于验证配置、界面流程和规则，不能证明真实机械或光学安全性。

## 连接与运行模式

程序启动后先进入 Welcome / Connection Center：选择 `Simulation` 或 `Real Hardware`，选择启用 Channel 数量（1、2 或 3），加载配置，然后连接。Simulation 会保留串口选择但不会打开串口；真实模式要求每个启用 Channel 选择不同且已枚举的 COM 口。连接成功后进入 Main UI，断开后返回 Welcome。

运行模式和串口选择保存在 `%PROGRAMDATA%\RSIC\FF\ff_gui.ini`。默认机台配置为 `%PROGRAMDATA%\RSIC\FF\ff_config.json`；首次安装时从安装目录的 `ff_config.default.json` 创建。配置、Training 会话和日志的默认根目录均为 `%PROGRAMDATA%\RSIC\FF`，升级或卸载不会删除这些数据。

## 配置与安全数据

正式配置为 UTF-8 JSON schema v2；CSV 模板仅用于编辑/交换，可使用随程序发布的 `config/` 目录转换。每台机台必须有 CH1、CH2、CH3，每个 Channel 有独立控制器和 Rod 列表；JSON 中的串口是可编辑的默认值，实际连接时由 Welcome 页为启用的 Channel 注入并校验，Simulation 和未启用 Channel 不要求 JSON 串口非空或唯一。Rod 地址始终是 `(ChannelId, RodId)` 的复合地址，不能把不同 Channel 的相同 Rod ID 混为一谈。

配置中的 `hard_min/max`、`soft_min/max`、`zero_position`、`safe_position` 和 `position_tolerance` 使用光栅尺计数（count）。光学位置使用 mm，由每根杆的 `P0`（mm）和 `P1`（mm/count）按 `P0 + P1 × count` 换算；`minimum_clearance_mm` 也是光学 mm。顶层 `motor_parameters` 供三通道共用：`speed_limit_raw` 为设备原始值 1–1023，`acceleration_raw` 和 `deceleration_raw` 为设备原始值 0–255；它们不是 count/s 或 count/s²。`rod_motion` 只保存软件到位容差（count）、稳定时间和运动超时（ms）。GUI 的 System 区域统一编辑这些参数。

真实硬件连接时，程序逐杆读取并核对设备短/长行程限位、Speed Limit、Acceleration 和 Deceleration；不同才写入，再回读确认，最后设置 Goal Speed。设备行程限位只会自动收紧；若配置要求扩大现有设备行程，连接失败并报告杆号，需先核对该电机的物理规格。前四项属于设备非易失参数，写入后断电仍保留；Simulation 不写设备。软件软限位、安全位、`P0/P1`、拓扑、间隙规则、到位容差及时间参数不会写入电机。`zero_position` 和 `inward_direction` 目前只保存、加载，没有参与运动计算，可视为待清理的兼容字段。旧配置中的逐杆运动字段仍可读取；若各杆数值不一致，保存时保留逐杆字段以免丢失差异。`FF_MOTION_SPEED` 仅为旧接口的兼容常量，不再覆盖配置的 `speed_limit_raw`。

真实运动前，应在配置中完成并复核下列内容。相邻 Rod 的间隙判断按光学位置执行，并允许 0.10 mm 的测量/位置反馈偏差：实际间隙比配置值少不超过 0.10 mm 时通过，超过 0.10 mm 时拒绝。

- 每根 Rod 的硬限位、软限位、零位、允许安全位及其有效标志；
- P0/P1 光学标定（`optical_position = P0 + P1 * encoder_position`）；
- `rod_layers.csv` / `layer_members` 的逐层角色和双向邻接关系；
- 所有相邻对的 `pair_clearance_rules.csv` 间隙规则；
- 禁止区、最大速度、到位容差、稳定时间和运动超时。

旧 schema v1 可以作为兼容输入读取，但缺少上述安全信息的配置不得用于真实运动。

## 目标、规划与停止

Main UI 中显示和编辑的是光学坐标；程序会按各 Rod 的 P0/P1 换算到编码器坐标。发送目标前会刷新状态，再检查复合地址、重复目标、限位、禁止区、层拓扑和相邻间隙。检查失败不会发送该组目标。规划预览产生各杆目标步骤；多杆执行按通道成组下发，并等待组内每根杆进入容差且保持到稳定时间，超时或出现报警即停止并报告失败。多杆规划只检查当前位置和目标位置组合，不验证连续联合轨迹。速度使用配置的 `motor_parameters.speed_limit_raw`，连接时逐杆设置。

当前 MightyZap 后端在连接初始化时设置一次各杆速度，运动时按杆连续写入目标。即使界面一次提交多根 Rod，也只是软件级成组提交，**不是**硬实时同步启动。`Stop` 与 `Emergency Stop` 是软件动作，不能替代经过认证的独立硬件急停回路。

## Training

Training 的扫描杆由 Channel 配置 `training_rod_id` 唯一决定，且必须是本 Channel 的 Layer 2 Rod。JSON 未配置时使用 `null`，CSV 的 `channels.csv` 留空；这种情况下该 Channel 的 Training 扫描和训练不可用。界面不会将训练杆作为 INI 用户偏好保存。

Training 支持 Sim1/Sim2 模拟扫描、谷值候选编辑、Layer 2 Rod 分配、保存与加载 `FF-TRAIN-1` 会话。扫描在反馈进入配置的位置容差后，按 `Scan Settle (ms)` 采样停顿；默认 20 ms，可在 Training 窗口调整，且不影响普通运动计划使用的 `stable_time_ms`。训练单步下发前只刷新执行杆及其直接层邻杆，到位等待只刷新训练杆，避免整机串口轮询掩盖设定的采样等待时间。会话包含机台/配置版本、样本、候选、目标和观测状态；加载后必须重新校验当前机台与配置版本，且执行仍会经过普通运动安全校验。当前发布不含经过验证的 Itek 相机 DLL 或相机配置，Real 能量采集不可用。

在 Training 窗口点击 **Save Session** 会立即写入 `%PROGRAMDATA%\RSIC\FF\training_sessions`，并在界面显示保存的完整文件路径或错误原因；无需再在文件对话框中确认。重复保存时会自动加数字后缀，避免覆盖同名文件。**Load Session** 仍通过文件对话框选择会话。

新保存的 Training 文件使用 `FF-TRAIN-2`，固定列出 CH1、CH2、CH3 各自的训练记录；未记录的通道为空对象。样本、候选、分配和杆目标逐项分行，所有小数最多三位。旧 `FF-TRAIN-1` 文件可加载，默认目录中的旧文件可用 `tools/migrate_training_sessions.py` 转换；脚本为每个文件保留 `.ff-train-1.bak` 备份。

完整 SDK 接口与实施边界见安装目录的 `FF_SDK_Reference.md`。
