# FF 应用层 SDK 1.0.1 版本记录

**状态：未正式发布。** FF GUI 与应用层 SDK 的当前本地版本均为 `1.0.1`。此前开发构建使用过的版本号不作为正式发布历史，也不提供跨开发构建的 ABI 兼容承诺。

## 交付内容

- `DX_FF.h` 对象接口与 `DX_FF_Types.h` 数据结构定义。
- Windows x64 / Visual Studio 2019 v142 / C++17 的 `ff_application.lib` 和 `ff_application.dll`。
- MightyZap 运行时 DLL、配置样例、接口文档与独立 SDK 示例。

## 发布前验证

```powershell
powershell -ExecutionPolicy Bypass -File tools/package_ff_sdk.ps1 -Configuration Release
.\x64\Release\ff_tests.exe
```

真实硬件上线前仍需独立完成机械限位、速度、互锁和安全边界验证；Simulation 只验证软件流程。
