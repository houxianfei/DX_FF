# Training 计算流程

1. 实例化 `FF::ff_driver driver;`。调用方自行完成图像采集与能量计算，并将位置（毫米）和对应能量组成 `Sample` 序列。SDK 不处理图像像素。
2. 使用 `driver.find_valleys(samples, settings)` 寻找遮挡导致的**能量谷值**；`interpolate_energy()` 和 `drag_interval()` 可用于交互编辑。
3. 调用 `driver.assign_training()` 计算并检查 Layer 2 杆位分配。输入必须属于当前机台的有效毫米坐标。扫描时使用 `driver.move_rod(address, encoder, velocity)` 单杆移动，内部检查本杆与直接邻杆并等待稳定。
4. 将分配结果和其他启用杆位置组成完整目标集，通过 `set_target_positions()` 保存，再调用 `plan_positions()` 和 `execute_positions()`；安全检查由底层自动完成。
5. 会话可用 `driver.save_session()`、`load_session()` 保存和读取 `FF-TRAIN-1` JSON。真实相机采集与扫描时序由应用负责。
