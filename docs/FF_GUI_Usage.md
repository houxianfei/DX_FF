# FF GUI 1.0.1 操作说明（未正式发布）

FF GUI 1.0.1 默认使用 **Simulation** 模式。它与 FF 应用层 SDK 1.0.1 共用 JSON/CSV 配置、目标校验和安全规则；Simulation 用于验证配置、界面流程和规则，不能证明真实机械或光学安全性。

## 连接与运行模式

程序启动后先进入 Welcome / Connection Center：选择 `Simulation` 或 `Real Hardware`，勾选 CH1、CH2、CH3 中需要启用的 1–3 个通道，加载配置并 Open。Open 检查配置和连接；之后点击 **Initialize**，使所有启用电机运动到 slot0。slot0 必须包含全部启用通道的全部电机。初始化前可以安全手动运动并保存位置，以便建立 slot0；扫描和训练执行必须先完成本次连接的初始化。Simulation 会保留串口选择但不会打开串口；真实模式要求每个启用 Channel 选择不同且已枚举的 COM 口。

运行模式和串口选择保存在 `%PROGRAMDATA%\RSIC\FF\ff_gui.ini`。默认机台配置为 `%PROGRAMDATA%\RSIC\FF\ff_config.json`；首次安装时从安装目录的 `ff_config.default.json` 创建。位置文件保存在 `position_presets`，Training 会话保存在 `training_sessions`，均位于 `%PROGRAMDATA%\RSIC\FF`。覆盖安装和卸载保留该目录中的配置、位置文件、会话和日志；安装目录中已编辑的 CSV 配置模板也不会被覆盖。

首次运行默认只勾选 CH1，因为交付的机台配置中 CH2/CH3 尚未具备完整机械拓扑。旧版只保存通道数量的 INI 会迁移到 CH1 默认选择；已有配置文件不会被覆盖。需要启用 CH2/CH3 时，先补齐对应通道的层归属、邻接关系和间隙规则，再显式勾选。选择了不完整通道时，Open 会显示具体 CH、Rod 和缺失项并拒绝连接。

## 配置与安全数据

正式配置为 UTF-8 JSON schema v2；CSV 模板仅用于编辑/交换，可使用随程序发布的 `config/` 目录转换。JSON 可以只包含已配置的通道；每个启用通道必须有电机列表、有效层归属、互相对应的邻接关系及每对邻接杆的间隙规则。缺少启用通道或存在异常数值时 Open 失败。未启用通道无需配置机械规则，也不会连接或运动。JSON 中的串口是可编辑的默认值，实际连接时由 Welcome 页为启用的 Channel 注入并校验。Rod 地址始终是 `(ChannelId, RodId)` 的复合地址。

配置中的 `hard_min/max`、`soft_min/max`、`zero_position`、`safe_position` 和 `position_tolerance` 使用光栅尺计数（count）。光学位置使用 mm，由每根杆的 `P0`（mm）和 `P1`（mm/count）按 `P0 + P1 × count` 换算；`minimum_clearance_mm` 也是光学 mm。顶层 `motor_parameters` 供三通道共用：`speed_limit_raw` 为设备原始值 1–1023，`acceleration_raw` 和 `deceleration_raw` 为设备原始值 0–255；它们不是 count/s 或 count/s²。`rod_motion` 只保存软件到位容差（count）、稳定时间和运动超时（ms）。GUI 的 System 区域统一编辑这些参数。

真实硬件连接时，程序逐杆读取并核对设备短/长行程限位、Speed Limit、Acceleration 和 Deceleration；不同才写入，再回读确认，最后设置 Goal Speed。设备行程限位只会自动收紧；若配置要求扩大现有设备行程，连接失败并报告杆号，需先核对该电机的物理规格。前四项属于设备非易失参数，写入后断电仍保留；Simulation 不写设备。软件软限位、安全位、`P0/P1`、拓扑、间隙规则、到位容差及时间参数不会写入电机。`zero_position` 和 `inward_direction` 目前只保存、加载，没有参与运动计算。旧配置中的逐杆通用运动参数只有在所有杆数值一致时才可迁移；不一致时需先修正配置。运动速度统一读取配置的 `motor_parameters.speed_limit_raw`。

真实运动前，应在配置中完成并复核下列内容。相邻 Rod 的间隙判断按光学位置执行，并允许 0.10 mm 的测量/位置反馈偏差：实际间隙比配置值少不超过 0.10 mm 时通过，超过 0.10 mm 时拒绝。

- 每根 Rod 的硬限位、软限位、零位、允许安全位及其有效标志；
- P0/P1 光学标定（`optical_position = P0 + P1 * encoder_position`）；
- `rod_layers.csv` / `layer_members` 的逐层角色和双向邻接关系；
- 所有相邻对的 `pair_clearance_rules.csv` 间隙规则；
- 禁止区、最大速度、到位容差、稳定时间和运动超时。

旧 schema v1 可以作为兼容输入读取，但缺少上述安全信息的配置不得用于真实运动。

## 目标、规划与停止

主界面的 Control 位于 CH tab 上方，位置 Slot 的保存和加载作用于当前全部启用通道；只有下方的滑杆随 CH tab 切换。新保存的位置文件使用 `FF-POS-2`，只包含启用通道。加载文件允许缺少整个通道，缺失通道保持原地；文件中存在的启用通道必须包含其全部电机。未启用通道会被忽略并在界面提示。slot0 的初始化始终执行更严格的全部启用电机完整性检查。旧 `FF-POS-1` 和 CSV 位置文件继续可读。

Main UI 中显示和编辑的是光学坐标；程序会按各 Rod 的 P0/P1 换算到编码器坐标。发送目标前会刷新状态，再检查复合地址、重复目标、限位、禁止区、层拓扑和相邻间隙。检查失败不会发送该组目标。规划预览产生各杆目标步骤；多杆执行按通道成组下发，并等待组内每根杆进入容差且保持到稳定时间，超时或出现报警即停止并报告失败。多杆规划只检查当前位置和目标位置组合，不验证连续联合轨迹。速度使用配置的 `motor_parameters.speed_limit_raw`，连接时逐杆设置。

当前 MightyZap 后端在连接初始化时设置一次各杆速度，运动时按杆连续写入目标。即使界面一次提交多根 Rod，也只是软件级成组提交，**不是**硬实时同步启动。`Stop` 与 `Emergency Stop` 是软件动作，不能替代经过认证的独立硬件急停回路。

## Training

Training 的扫描杆由 Channel 配置 `training_rod_id` 唯一决定，且必须是本 Channel 的 Layer 2 Rod。JSON 未配置时使用 `null`，CSV 的 `channels.csv` 留空；这种情况下该 Channel 的 Training 扫描和训练不可用。界面不会将训练杆作为 INI 用户偏好保存。

Training 在 tab 上方为每个启用通道显示数据源、训练杆、扫描起点和终点及谷值参数。采样数量为全局设置，默认 300，包含两个端点；各通道可使用不同范围和方向。每轮同时下发各通道目标，等待所有训练杆进入容差并稳定后才追加该轮样本，所以各通道样本数量一致。`Scan Settle (ms)` 默认 20 ms。扫描中可切换 CH tab 查看曲线和结果，但设置与候选编辑冻结。**Find Valleys**、**Clear Lines** 和 **Plan** 可作用于全部或指定 CH；**Execute** 汇总所有启用 CH 的有效计划，无候选通道保持当前位置。当前发布不含经过验证的 Itek 相机 DLL 或相机配置，Real 能量采集不可用。

在 Training 窗口点击 **Save Session** 会立即写入 `%PROGRAMDATA%\RSIC\FF\training_sessions`，并在界面显示保存的完整文件路径或错误原因；无需再在文件对话框中确认。重复保存时会自动加数字后缀，避免覆盖同名文件。**Load Session** 仍通过文件对话框选择会话。

新保存的 Training 文件使用 `FF-TRAIN-3`，只列出当前启用的通道，并记录采样数量。样本、候选、分配和杆目标逐项分行，所有小数最多三位。旧 `FF-TRAIN-1/2` 文件仍可加载；加载后须重新 Plan 才能执行。

## 按业务使用

- 首次调试：Open → 检查反馈 → 手动调整并复核杆位 → 保存 Slot 0。不要用当前位置自动覆盖既有 slot0。
- 日常开机：选择实际启用通道 → Open → Initialize → 确认全部到位后切换配方或 Training。
- 仿真训练：配置每 CH 范围与数据源、全局采样数 → Start Scan → Find Valleys → 编辑候选 → Plan → 检查全部目标及避让 → Execute → Save Session。各步失败先查看具体原因，不跳过校验。
- 结果复用：Load Session → 检查机台、配置及候选 → 重新 Plan → Execute；要恢复已确认的实际杆位，使用另存的位置 Slot。
- 停止恢复：Stop/急停/故障复位后，检查真实反馈与安全状态，再重新 Initialize。Stop Scan 保留部分样本，不自动续扫。

当前 GUI 不控制 stage/TDI，不能用仿真扫描代替真实光学训练。真实 Training 由集成应用执行“每个训练杆位置重复扫描同一区域”，详见源码 docs/public_sdk 下的《应用场景指南》和《Training计算流程》。SDK 发布包内相应文档位于 docs。GUI 程序包独立交付本操作说明，集成文档随 SDK 获取。
