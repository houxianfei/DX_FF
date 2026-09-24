param(
  [string]$Configuration = "Release",
  [string]$Platform = "x64",
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$out = Join-Path $root "FF_SDK"
$build = Join-Path $root "$Platform\$Configuration"
$msbuild = "D:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe"

if (!$SkipBuild) {
  & $msbuild (Join-Path $root "FF.sln") /p:Configuration=$Configuration /p:Platform=$Platform /p:ImportUserProps=false /m
  if ($LASTEXITCODE) { throw "MSBuild failed" }
}

$library = Join-Path $build "ff_core.lib"
if (!(Test-Path $library)) { throw "SDK library was not found: $library" }

# FF_SDK is the designated generated release target.  Recreate it so no
# removed header or stale binary can be shipped with the current SDK.
if (Test-Path $out) { Remove-Item $out -Recurse -Force }
New-Item $out -ItemType Directory | Out-Null
New-Item "$out\include\ff" -ItemType Directory -Force | Out-Null
New-Item "$out\lib\$Platform\$Configuration" -ItemType Directory -Force | Out-Null
New-Item "$out\bin\$Platform\$Configuration" -ItemType Directory -Force | Out-Null
New-Item "$out\docs" -ItemType Directory -Force | Out-Null
New-Item "$out\config" -ItemType Directory -Force | Out-Null

Copy-Item "$root\include\ff\FF.hpp" "$out\include\ff\FF.hpp"
Copy-Item "$root\include\ff\Training.hpp" "$out\include\ff\Training.hpp"
Copy-Item $library "$out\lib\$Platform\$Configuration\ff_core.lib"
Copy-Item "$root\third_party\MightyZap\bin\CppDllMightyZap_Ver2010.dll" "$out\bin\$Platform\$Configuration\CppDllMightyZap_Ver2010.dll"
Copy-Item "$root\third_party\MightyZap\include\CppDllMightyZap_Ver2010.h" "$out\include\CppDllMightyZap_Ver2010.h"
Copy-Item "$root\third_party\MightyZap\lib\CppDllMightyZap_Ver2010.lib" "$out\lib\$Platform\$Configuration\CppDllMightyZap_Ver2010.lib"
$referenceDoc = Get-ChildItem (Join-Path $root "docs\architecture") -Filter "*.md" | Select-Object -First 1
if (!$referenceDoc) { throw "SDK reference document was not found" }
Copy-Item $referenceDoc.FullName "$out\docs\FF_SDK_Reference.md"
Copy-Item "$root\docs\SDK_RELEASE_NOTES.md" "$out\docs\SDK_RELEASE_NOTES.md"
Copy-Item "$root\config\defaults\ff_config.json" "$out\config\ff_config.example.json"
Copy-Item "$root\config\templates\*" "$out\config" -Recurse
Copy-Item "$root\docs\SDK_PACKAGE_README.md" "$out\README.md"
Set-Content -Path "$out\VERSION" -Value "1.1.0" -NoNewline

Write-Host "SDK published: $out"
