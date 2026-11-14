param([string]$Configuration="Release",[string]$Platform="x64",[switch]$SkipBuild)
$ErrorActionPreference="Stop"
$root=(Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$out=Join-Path $root "publish\ff_gui_app"
$build=Join-Path $root "$Platform\$Configuration"
$sdkBin=Join-Path $root "FF_SDK\bin\$Platform\$Configuration"
$vswhere=Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
$msbuild=& $vswhere -products '*' -version '[16.0,17.0)' -find 'MSBuild\**\Bin\MSBuild.exe' | Select-Object -First 1
if(!$msbuild){throw "Visual Studio 2019 MSBuild was not found"}
if(!$SkipBuild){
  & $msbuild (Join-Path $root "ff_gui.vcxproj") /p:Configuration=$Configuration /p:Platform=$Platform /p:ImportUserProps=false /m
  if($LASTEXITCODE){throw "GUI build failed"}
}
foreach($required in @("$build\ff_gui.exe","$sdkBin\ff_application.dll","$sdkBin\CppDllMightyZap_Ver2010.dll")){
  if(!(Test-Path $required)){throw "Package dependency was not found: $required"}
}
$resolvedRoot=[IO.Path]::GetFullPath($root)
$resolvedOut=[IO.Path]::GetFullPath($out)
if(!$resolvedOut.StartsWith(($resolvedRoot+[IO.Path]::DirectorySeparatorChar),[StringComparison]::OrdinalIgnoreCase)){throw "Package target is outside the repository"}
if(Test-Path $out){Remove-Item -LiteralPath $out -Recurse -Force}
New-Item $out -ItemType Directory | Out-Null
Copy-Item "$build\ff_gui.exe" $out
Copy-Item "$sdkBin\*.dll" $out
Copy-Item "$root\config\defaults\ff_config.json" "$out\ff_config.default.json"
Copy-Item "$root\docs\FF_GUI_Usage.md" "$out\FF_GUI_Usage.md"
New-Item "$out\config" -ItemType Directory | Out-Null
Copy-Item "$root\config\templates\*" "$out\config" -Recurse
$redist=Get-ChildItem 'C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Redist\MSVC' -Recurse -Filter 'vc_redist.x64.exe' -ErrorAction SilentlyContinue | Select-Object -First 1
if($redist){Copy-Item $redist.FullName "$out\vc_redist.x64.exe"}
Write-Host "Published: $out"
