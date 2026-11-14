param(
  [string]$Configuration = "Release",
  [string]$Platform = "x64",
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$out = Join-Path $root "FF_SDK"
$build = Join-Path $root "$Platform\$Configuration"
$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (!(Test-Path $vswhere)) { throw "Visual Studio Installer (vswhere.exe) was not found" }
$msbuild = & $vswhere -products '*' -version '[16.0,17.0)' -find 'MSBuild\**\Bin\MSBuild.exe' | Select-Object -First 1
if (!$msbuild) { throw "Visual Studio 2019 MSBuild was not found" }

if (!$SkipBuild) {
  & $msbuild (Join-Path $root "ff_application.vcxproj") /p:Configuration=$Configuration /p:Platform=$Platform /p:ImportUserProps=false /m
  if ($LASTEXITCODE) { throw "MSBuild failed" }
}

$library = Join-Path $build "ff_application.lib"
$dll = Join-Path $build "ff_application.dll"
foreach ($required in @($library, $dll, "$root\third_party\MightyZap\bin\CppDllMightyZap_Ver2010.dll")) {
  if (!(Test-Path $required)) { throw "SDK dependency was not found: $required" }
}

# FF_SDK is the designated generated release target.  Recreate it so no
# removed header or stale binary can be shipped with the current SDK.
$resolvedRoot = [IO.Path]::GetFullPath($root.Path)
$resolvedOut = [IO.Path]::GetFullPath($out)
if (!$resolvedOut.StartsWith(($resolvedRoot + [IO.Path]::DirectorySeparatorChar), [StringComparison]::OrdinalIgnoreCase)) {
  throw "Release target is outside the repository: $resolvedOut"
}
if (Test-Path $out) { Remove-Item -LiteralPath $out -Recurse -Force }
New-Item $out -ItemType Directory | Out-Null
New-Item "$out\include" -ItemType Directory -Force | Out-Null
New-Item "$out\lib\$Platform\$Configuration" -ItemType Directory -Force | Out-Null
New-Item "$out\bin\$Platform\$Configuration" -ItemType Directory -Force | Out-Null
New-Item "$out\docs" -ItemType Directory -Force | Out-Null
New-Item "$out\config" -ItemType Directory -Force | Out-Null

Copy-Item "$root\include\DX_FF.h" "$out\include\DX_FF.h"
Copy-Item "$root\include\DX_FF_Types.h" "$out\include\DX_FF_Types.h"
Copy-Item $library "$out\lib\$Platform\$Configuration\ff_application.lib"
Copy-Item $dll "$out\bin\$Platform\$Configuration\ff_application.dll"
Copy-Item "$root\third_party\MightyZap\bin\CppDllMightyZap_Ver2010.dll" "$out\bin\$Platform\$Configuration\CppDllMightyZap_Ver2010.dll"
Copy-Item "$root\docs\public_sdk\*" "$out\docs" -Recurse
New-Item "$out\examples" -ItemType Directory | Out-Null
Copy-Item "$root\examples\public_sdk\*" "$out\examples" -Recurse
Copy-Item "$root\config\defaults\ff_config.json" "$out\config\ff_config.example.json"
Copy-Item "$root\config\templates\*" "$out\config" -Recurse
$quickStart = Get-ChildItem "$root\docs\public_sdk" -File | Where-Object { $_.BaseName -match '^\u5feb\u901f\u5f00\u59cb$' } | Select-Object -First 1
if (!$quickStart) { throw "SDK quick start was not found" }
$quickStartText = Get-Content -LiteralPath $quickStart.FullName -Raw -Encoding UTF8
$quickStartText = $quickStartText -replace '\]\(([^)#]+\.md)\)', '](docs/$1)'
[IO.File]::WriteAllText((Join-Path $out 'README.md'), $quickStartText, (New-Object Text.UTF8Encoding($false)))
Set-Content -Path "$out\VERSION" -Value "1.0.1" -NoNewline

Write-Host "SDK published: $out"
