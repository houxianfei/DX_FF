param(
  [ValidateSet('Release')][string]$Configuration = 'Release',
  [switch]$SkipPackage
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$script = Join-Path $root 'installer\ff_gui.iss'

function Read-Version([string]$Path, [string]$Pattern) {
  $contents = [IO.File]::ReadAllText($Path)
  $match = [regex]::Match($contents, $Pattern)
  if (!$match.Success) { throw "Version was not found in $Path" }
  return $match.Groups[1].Value
}

$guiVersion = Read-Version (Join-Path $root 'app\app_version.h') '#define\s+FF_GUI_VERSION\s+"([0-9]+\.[0-9]+\.[0-9]+)"'
$sdkVersion = Read-Version (Join-Path $root 'include\ff\Version.hpp') '#define\s+FF_SDK_VERSION\s+"([0-9]+\.[0-9]+\.[0-9]+)"'
$installerVersion = Read-Version $script '#define\s+MyAppVersion\s+"([0-9]+\.[0-9]+\.[0-9]+)"'
$resourceVersion = Read-Version (Join-Path $root 'app\resources\ff_gui.rc') 'VALUE\s+"ProductVersion",\s+"([0-9]+\.[0-9]+\.[0-9]+)\\0"'
if ($guiVersion -ne $sdkVersion -or $guiVersion -ne $installerVersion -or $guiVersion -ne $resourceVersion) {
  throw "Version mismatch: GUI=$guiVersion SDK=$sdkVersion installer=$installerVersion resource=$resourceVersion"
}

if (!$SkipPackage) {
  & (Join-Path $root 'tools\package_ff_gui_app.ps1') -Configuration $Configuration
  if ($LASTEXITCODE) { throw 'Application package failed' }
}

$candidates = @(
  (Get-Command ISCC.exe -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Source),
  (Join-Path ${env:ProgramFiles(x86)} 'Inno Setup 6\ISCC.exe'),
  (Join-Path $env:LOCALAPPDATA 'Programs\Inno Setup 6\ISCC.exe')
)
$iscc = $candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
if (!$iscc) { throw 'Inno Setup 6 ISCC.exe was not found' }

& $iscc $script
if ($LASTEXITCODE) { throw "Installer compilation failed: $LASTEXITCODE" }

$output = Join-Path $root "publish\installer\FF_GUI_Setup_v$guiVersion.exe"
if (!(Test-Path -LiteralPath $output)) { throw "Installer was not created: $output" }
Write-Output "Installer: $output"
