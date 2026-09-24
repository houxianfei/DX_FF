# FF C++ SDK 1.1.0

此目录由 `tools/package_ff_sdk.ps1` 生成，是 Windows x64、Visual Studio 2019 v142、C++17 的 FF 集成包。

## 内容

| 路径 | 内容 |
| --- | --- |
| `include/ff/FF.hpp` | 整机、配置、状态、后端和应用服务公共接口 |
| `include/ff/Training.hpp` | Training 分析、会话和能量源接口 |
| `lib/x64/Release/ff_core.lib` | FF 静态库 |
| `bin/x64/Release/CppDllMightyZap_Ver2010.dll` | 真实硬件后端所需厂商 DLL |
| `config/` | JSON 例子和可编辑 CSV 配置模板 |
| `docs/` | 接口参考、运动安全约束和发布说明 |

## 集成

1. 将 `include` 加入附加包含目录；将 `lib/x64/Release` 加入库目录并链接 `ff_core.lib`。
2. 使用 `#include <ff/FF.hpp>`；Training 功能另加 `#include <ff/Training.hpp>`。
3. 使用 `FFConfigIO::load()` 读取经校验的 JSON，随后为 CH1、CH2、CH3 各提供一个后端并构造 `ff::FF`。
4. 先 `initialize()`、`connect()`，再提交目标；每次命令成功仅表示已接受，不表示已稳定到位。
5. 真实硬件运行时，将 `CppDllMightyZap_Ver2010.dll` 部署到应用程序目录，并按厂商许可证处理其头文件、导入库和 DLL。

最小 Simulation 示例：

```cpp
#include <array>
#include <memory>
#include <ff/FF.hpp>

auto loaded = ff::FF::loadConfig("config/ff_config.example.json");
if (!loaded) return 1;
std::array<std::unique_ptr<ff::IControllerBackend>, 3> backends;
for (auto& backend : backends) backend = ff::makeSimulationBackend();
ff::FF machine(std::move(loaded.value), std::move(backends));
if (!machine.initialize() || !machine.connect()) return 2;
auto sent = machine.submitMotion({{{ff::ChannelId::CH1, 0}, 1200, 20.0}, false});
if (!sent) return 3;
machine.disconnect();
```

详细的配置格式、错误处理、Training 会话和当前安全限制见 `docs/FF_SDK_Reference.md`。真实机台投入运行前，必须完成独立的硬件安全评审；SDK 的软件急停不替代硬件安全回路。
