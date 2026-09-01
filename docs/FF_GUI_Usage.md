# FF GUI 1.0.0

首版默认运行在 Simulation 模式。加载完整机台配置后，连接、状态刷新、目标编辑、规划预览和执行均可在无硬件环境验证。

真实执行前必须补齐每根 Rod 的软/硬限位、安全位、层拓扑、禁区和间隙规则。旧 schema v1 配置可读取，但缺少安全数据时不得用于真实运动。

Training 使用每个 Channel 配置中的 `training_rod_id` 确定扫描训练杆；该值必须是同一 Channel 的 Layer 2 Rod。JSON 中未配置时写 `null`，CSV 模板 `channels.csv` 中留空。Training UI 不提供手动选择，也不从 INI 恢复训练杆。

配置和目标文件默认保存到 `%PROGRAMDATA%\RSIC\FF`，日志目录为 `%PROGRAMDATA%\RSIC\FF\logs`。安装升级和卸载不会删除这些用户数据。
