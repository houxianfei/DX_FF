# Training 计算与执行流程

调用顺序：**打开 → 扫描取样 → 寻谷 → 位置分配 → 合成完整目标集 → 多杆运动 → 记录位置 → 关闭**。调用方负责图像采集、能量计算和扫描时序；SDK 不处理像素。扫描杆用 `move_rod()` 单杆移动，内部检查相邻杆边界并等待稳定。

以下代码展示扫描数据已经由调用方收集后的处理。`samples` 中的每项由采样时的光学毫米位置和图像能量组成；完整扫描循环在每个采样点调用 `move_rod()`、采集图像并追加 `Sample`。所有失败路径应记录本次结果的 `code/message`，必要时刷新反馈后再决定动作。

```cpp
FF::ff_driver driver;
auto opened = driver.open("config.json", FF::Mode::Simulation);
if (!opened) return 1;

// 扫描循环：driver.move_rod(scanRod, encoder); 采集图像、计算能量并追加样本。
std::vector<FF::Sample> samples; // 由上面的扫描循环填充
FF::ValleySettings settings;
auto valleysMm = driver.find_valleys(samples, settings);
auto assigned = driver.assign_training(0, valleysMm);
if (!assigned) { driver.close(); return 2; }

auto refreshed = driver.refresh();
if (!refreshed) { driver.close(); return 3; }
std::vector<FF::Target> targets;
for (const auto& current : driver.m_current_positions) {
  if (!current.valid) { driver.close(); return 4; }
  targets.push_back({current.address, current.encoder}); // 未分配的杆保持此位置
}
for (const auto& aim : assigned.value.aims) {
  if (!aim.assigned) continue;
  for (auto& target : targets)
    if (target.address.channel == aim.address.channel &&
        target.address.rod == aim.address.rod)
      target.position = aim.encoderAim;
}
auto moved = driver.move_rods(targets);
if (!moved) { driver.refresh(); driver.close(); return 5; }
auto saved = driver.record_positions("trained.json", "training complete");
if (!saved) { driver.close(); return 6; }
driver.close();
```

`assign_training()` 给出 `Aim.encoderAim`，并不下发运动。`move_rods()` 要求全部启用杆各有一个目标，运动前根据最新反馈检查起点和终点组合；同通道尽快成组下发，不保证严格同步启动。`send_rods()` 可用于非阻塞下发，成功仅表示下发成功，须自行 `refresh()` 并确认到位。`interpolate_energy()`、`drag_interval()` 可辅助编辑候选点。会话可用 `save_training_session()`、`load_training_session()` 或原始 JSON 接口保存与读取。

三通道记录使用 `TrainingArchive` 和 `save_training_archive()` / `load_training_archive()`。`FF-TRAIN-2` 固定列出三个通道，未记录通道写 `{}`；样本、候选点、分配结果、杆状态和执行目标逐项分行。所有浮点字段四舍五入到最多三位小数，整数能量和编码器值不变。旧 `FF-TRAIN-1` 仍可读取并映射到原通道。

新文件的结构如下，实际扫描的每个样本和每根杆均独占一项：

```json
{
  "schema": "FF-TRAIN-2",
  "machine_id": "FF-001",
  "config_version": "FF-CONFIG-1.1",
  "created_unix_seconds": 1794832729,
  "selected_channel": 0,
  "channels": [
    {"channel": 0, "training": {
      "source": "Sim1",
      "training_rod": 12,
      "occlusion_summary": "CH1 training",
      "start_mm": 2.198, "end_mm": 87.912, "step_mm": 0.1,
      "complete": true,
      "valley": {"smoothing": 5, "prominence": 0.1, "distance_mm": 0.5, "maximum": 14},
      "samples": [
        {"position_mm": 2.197, "energy": 999996},
        {"position_mm": 2.307, "energy": 1000886}
      ],
      "candidates_mm": [
        23.401
      ],
      "aims": [],
      "rod_states": [],
      "execution_targets": []
    }},
    {"channel": 1, "training": {}},
    {"channel": 2, "training": {}}
  ]
}
```
