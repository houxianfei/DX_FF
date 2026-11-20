# FF 应用层 SDK 1.0.1 版本记录

**状态：本地交付构建，尚未正式对外发布。** GUI 与 SDK 当前均为 1.0.1，各自独立标识；不承诺与早期开发构建的 ABI 兼容。

## 交付内容

- FF_SDK_v1.0.1.zip，解压目录同名；Windows x64 / VS2019 v142 / C++17。
- 两个公开头文件 DX_FF.h、DX_FF_Types.h，导入库 ff_application.lib，ff_application.dll 及 MightyZap 运行时 DLL。
- Visual C++ x64 运行库安装程序、完整对外文档、配置/仿真样例、C++ 示例和 Python 配置转换工具。
- SDK 版本、VERSION 文件、压缩包名称共用内部版本定义；GUI 编译仅更新本地暂存目录。

## 当前接口与行为

连接后通过 initialize(slot0) 运动到初始杆位；slot0 覆盖全部启用电机。支持任意非空物理通道组合。停止、急停、复位和断开会清除初始化标志。

阻塞 move_rod/move_rods 等待到位稳定；非阻塞 send_rod/send_rods/send_partial_rods 仅表示下发。完整多杆方法要求全部启用杆目标，partial 方法仅向列出的杆发送。validate_rods 只校验不运动。

速度由 motor_parameters.speed_limit_raw 配置，连接时逐杆设置；目标不包含速度。旧固定速度宏及独立运动策略头文件已删除。

位置文件新写 FF-POS-2，允许普通配方缺少整个通道；load_positions 返回适用目标和摘要，不运动。多通道训练新写 FF-TRAIN-3，包含启用通道和采样数；旧格式继续可读。

Training 分配结果须包含候选及避让目标。stage/TDI 接口以外部适配伪代码说明，真实采集未接入 GUI；仿真不代表硬件或光学验收。

## 发布验证

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_sdk.ps1 -Configuration Release
```

发布后从 ZIP 解压目录编译和运行示例，核对 version() 与 VERSION、包名一致。真实投运前独立完成限位、速度、机械包络和互锁验证。
