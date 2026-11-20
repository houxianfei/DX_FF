param(
  [ValidateSet("Release", "Debug")][string]$Configuration = "Release",
  [ValidateSet("x64")][string]$Platform = "x64",
  [switch]$SkipBuild,
  [switch]$SkipArchive
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$out = Join-Path $root "FF_SDK"
$build = Join-Path $root "$Platform\$Configuration"
$versionHeader = Get-Content -LiteralPath (Join-Path $root "include\ff\Version.hpp") -Raw
$versionMatch = [regex]::Match($versionHeader, '(?m)^#define\s+FF_SDK_VERSION\s+"(\d+\.\d+\.\d+)"\s*$')
if (!$versionMatch.Success) { throw "SDK version must be defined as major.minor.patch in include/ff/Version.hpp" }
$version = $versionMatch.Groups[1].Value
$packageName = "FF_SDK_v$version"
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
# Publish Markdown documents only, without local editor metadata (e.g. .obsidian).
Get-ChildItem -LiteralPath "$root\docs\public_sdk" -Filter "*.md" -File | Copy-Item -Destination "$out\docs"
Copy-Item "$root\docs\FF_GUI_Usage.md" "$out\docs\FF_GUI_Usage.md"
Copy-Item "$root\docs\SDK_RELEASE_NOTES.md" "$out\RELEASE_NOTES.md"
$packageGuide = Get-Content -LiteralPath "$root\docs\SDK_PACKAGE_README.md" -Raw -Encoding UTF8
$packageGuide = $packageGuide.Replace('](public_sdk/', '](docs/')
[IO.File]::WriteAllText((Join-Path $out 'PACKAGE_README.md'), $packageGuide, (New-Object Text.UTF8Encoding($false)))
New-Item "$out\examples" -ItemType Directory | Out-Null
Copy-Item "$root\examples\public_sdk\*" "$out\examples" -Recurse
New-Item "$out\tools" -ItemType Directory | Out-Null
Copy-Item "$root\tools\ff_config_convert.py" "$out\tools\ff_config_convert.py"
Copy-Item "$root\config\defaults\ff_config.json" "$out\config\ff_config.example.json"
Copy-Item "$root\config\templates\*" "$out\config" -Recurse
if ($Configuration -eq "Release" -and !$SkipArchive) {
  $vsInstall = & $vswhere -products '*' -version '[16.0,17.0)' -property installationPath | Select-Object -First 1
  $redistRoot = Join-Path $vsInstall "VC\Redist\MSVC"
  $redist = Get-ChildItem -LiteralPath $redistRoot -Filter "vc_redist.x64.exe" -Recurse -File |
    Sort-Object FullName -Descending | Select-Object -First 1
  if (!$redist) { throw "Visual C++ x64 redistributable installer was not found in $redistRoot" }
  New-Item "$out\runtime" -ItemType Directory | Out-Null
  Copy-Item -LiteralPath $redist.FullName -Destination "$out\runtime\vc_redist.x64.exe"
}
$quickStart = Get-ChildItem "$root\docs\public_sdk" -File | Where-Object { $_.BaseName -match '^\u5feb\u901f\u5f00\u59cb$' } | Select-Object -First 1
if (!$quickStart) { throw "SDK quick start was not found" }
$quickStartText = Get-Content -LiteralPath $quickStart.FullName -Raw -Encoding UTF8
$quickStartText = $quickStartText -replace '\]\(([^)#]+\.md)\)', '](docs/$1)'
$quickStartText += "`r`n## Package contents`r`n`r`n[Package guide](PACKAGE_README.md) | [Release notes](RELEASE_NOTES.md) | [GUI operations](docs/FF_GUI_Usage.md)`r`n"
[IO.File]::WriteAllText((Join-Path $out 'README.md'), $quickStartText, (New-Object Text.UTF8Encoding($false)))
[IO.File]::WriteAllText((Join-Path $out 'VERSION'), $version, (New-Object Text.UTF8Encoding($false)))

Write-Host "SDK staging directory: $out ($Configuration, v$version)"
if ($SkipArchive -or $Configuration -ne "Release") {
  Write-Host "Archive skipped; versioned distribution archives are produced for Release builds only."
  exit 0
}

# Keep the stable FF_SDK path for local GUI builds. Ship a versioned root in ZIP.
$releaseRoot = Join-Path $root "publish\sdk"
New-Item -Path $releaseRoot -ItemType Directory -Force | Out-Null
$stage = Join-Path $releaseRoot (".staging-" + [Guid]::NewGuid().ToString("N"))
$temporaryZip = Join-Path $stage "$packageName.zip"
$archivePath = Join-Path $releaseRoot "$packageName.zip"
$resolvedStage = [IO.Path]::GetFullPath($stage)
$resolvedRelease = [IO.Path]::GetFullPath($releaseRoot)
if (!$resolvedRelease.StartsWith(($resolvedRoot + [IO.Path]::DirectorySeparatorChar), [StringComparison]::OrdinalIgnoreCase) -or
    !$resolvedStage.StartsWith(($resolvedRelease + [IO.Path]::DirectorySeparatorChar), [StringComparison]::OrdinalIgnoreCase)) {
  throw "Archive staging target is outside the workspace release directory"
}
New-Item -Path $stage -ItemType Directory | Out-Null
try {
  $versionedDirectory = Join-Path $stage $packageName
  Copy-Item -LiteralPath $out -Destination $versionedDirectory -Recurse
  Compress-Archive -LiteralPath $versionedDirectory -DestinationPath $temporaryZip -CompressionLevel Optimal
  # Replace this version only after the entire new archive has been created.
  Move-Item -LiteralPath $temporaryZip -Destination $archivePath -Force
  Write-Host "SDK archive published: $archivePath"
} finally {
  Remove-Item -LiteralPath $resolvedStage -Recurse -Force
}
