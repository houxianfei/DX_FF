# FF GUI 1.0.1 操作说明（未正式发布）

FF GUI 1.0.1 默认使用 **Simulation** 模式。它与 FF 应用层 SDK 1.0.1 共用 JSON/CSV 配置、目标校验和安全规则；Simulation 用于验证配置、界面流程和规则，不能证明真实机械或光学安全性。

## 连接与运行模式

程序启动后先进入 Welcome / Connection Center：选择 `Simulation` 或 `Real Hardware`，选择启用 Channel 数量（1、2 或 3），加载配置，然后连接。Simulation 会保留串口选择但不会打开串口；真实模式要求每个启用 Channel 选择不同且已枚举的 COM 口。连接成功后进入 Main UI，断开后返回 Welcome。

运行模式和串口选择保存在 `%PROGRAMDATA%\RSIC\FF\ff_gui.ini`。默认机台配置为 `%PROGRAMDATA%\RSIC\FF\ff_config.json`；首次安装时从安装目录的 `ff_config.default.json` 创建。配置、Training 会话和日志的默认根目录均为 `%PROGRAMDATA%\RSIC\FF`，升级或卸载不会删除这些数据。

## 配置与安全数据

正式配置为 UTF-8 JSON schema v2；CSV 模板仅用于编辑/交换，可使用随程序发布的 `config/` 目录转换。每台机台必须有 CH1、CH2、CH3，每个 Channel 有独立控制器和 Rod 列表；JSON 中的串口是可编辑的默认值，实际连接时由 Welcome 页为启用的 Channel 注入并校验，Simulation 和未启用 Channel 不要求 JSON 串口非空或唯一。Rod 地址始终是 `(ChannelId, RodId)` 的复合地址，不能把不同 Channel 的相同 Rod ID 混为一谈。

真实运动前，应在配置中完成并复核下列内容。相邻 Rod 的间隙判断按光学位置执行，并允许 0.10 mm 的测量/位置反馈偏差：实际间隙比配置值少不超过 0.10 mm 时通过，超过 0.10 mm 时拒绝。

- 每根 Rod 的硬限位、软限位、零位、允许安全位及其有效标志；
- P0/P1 光学标定（`optical_position = P0 + P1 * encoder_position`）；
- `rod_layers.csv` / `layer_members` 的逐层角色和双向邻接关系；
- 所有相邻对的 `pair_clearance_rules.csv` 间隙规则；
- 禁止区、最大速度、到位容差、稳定时间和运动超时。

旧 schema v1 可以作为兼容输入读取，但缺少上述安全信息的配置不得用于真实运动。

## 目标、规划与停止

Main UI 中显示和编辑的是光学坐标；程序会按各 Rod 的 P0/P1 换算到编码器坐标。发送目标前会刷新状态，再检查复合地址、重复目标、限位、禁止区、层拓扑和相邻间隙。检查失败不会发送该组目标。规划预览产生安全位和目标位的顺序步骤；执行会等待位置进入容差并保持到稳定时间，超时或出现报警即停止并报告失败。

当前 MightyZap 后端按杆顺序写入目标。即使界面一次提交多根 Rod，也只是软件级成组提交，**不是**硬实时同步启动。`Stop` 与 `Emergency Stop` 是软件动作，不能替代经过认证的独立硬件急停回路。

## Training

Training 的扫描杆由 Channel 配置 `training_rod_id` 唯一决定，且必须是本 Channel 的 Layer 2 Rod。JSON 未配置时使用 `null`，CSV 的 `channels.csv` 留空；这种情况下该 Channel 的 Training 扫描和训练不可用。界面不会将训练杆作为 INI 用户偏好保存。

Training 支持 Sim1/Sim2 模拟扫描、谷值候选编辑、Layer 2 Rod 分配、保存与加载 `FF-TRAIN-1` 会话。扫描在反馈进入配置的位置容差后，按 `Scan Settle (ms)` 采样停顿；默认 20 ms，可在 Training 窗口调整，且不影响普通运动计划使用的 `stable_time_ms`。训练单步下发前只刷新执行杆及其直接层邻杆，到位等待只刷新训练杆，避免整机串口轮询掩盖设定的采样等待时间。会话包含机台/配置版本、样本、候选、目标和观测状态；加载后必须重新校验当前机台与配置版本，且执行仍会经过普通运动安全校验。当前发布不含经过验证的 Itek 相机 DLL 或相机配置，Real 能量采集不可用。

完整 SDK 接口与实施边界见安装目录的 `FF_SDK_Reference.md`。
