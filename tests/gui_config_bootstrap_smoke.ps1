param([string]$AppDirectory = (Join-Path $PSScriptRoot '..\publish\ff_gui_app'))

$ErrorActionPreference = 'Stop'
$app = Join-Path $AppDirectory 'ff_gui.exe'
$bundle = Join-Path $AppDirectory 'ff_config.default.json'
if (!(Test-Path -LiteralPath $app) -or !(Test-Path -LiteralPath $bundle)) {
  throw 'Build and package the Release GUI before running this smoke test.'
}

$previousProgramData = $env:PROGRAMDATA
try {
  foreach ($legacy in @($false, $true)) {
    $root = Join-Path $PSScriptRoot ('..\x64\Release\config_bootstrap_smoke_' +
      [guid]::NewGuid().ToString('N'))
    $data = Join-Path $root 'RSIC\FF'
    New-Item -ItemType Directory -Path $data -Force | Out-Null
    $config = Join-Path $data 'ff_config.json'
    $ini = Join-Path $data 'ff_gui.ini'
    if ($legacy) {
      Copy-Item -LiteralPath $bundle -Destination $config
      [IO.File]::WriteAllText($ini, "mode=simulation`nenabled_channels=3`n")
    }
    $before = if ($legacy) { (Get-FileHash -LiteralPath $config).Hash } else { $null }
    $env:PROGRAMDATA = (Resolve-Path -LiteralPath $root).Path
    $process = Start-Process -FilePath $app -WindowStyle Hidden -PassThru
    try {
      for ($attempt = 0; $attempt -lt 50; ++$attempt) {
        $process.Refresh()
        if ($process.HasExited) { throw "GUI exited with code $($process.ExitCode)" }
        if ((Test-Path -LiteralPath $config) -and
            (Select-String -Path $ini -Pattern '^enabled_mask=100$' -Quiet -ErrorAction SilentlyContinue)) {
          break
        }
        Start-Sleep -Milliseconds 100
      }
      if (!(Test-Path -LiteralPath $config)) { throw 'Default configuration was not installed.' }
      if (!(Select-String -Path $ini -Pattern '^enabled_mask=100$' -Quiet)) {
        throw 'Channel selection did not migrate to CH1.'
      }
      $hash = (Get-FileHash -LiteralPath $config).Hash
      if ($legacy -and $hash -ne $before) { throw 'Existing configuration was overwritten.' }
      if (!$legacy -and $hash -ne (Get-FileHash -LiteralPath $bundle).Hash) {
        throw 'Installed configuration differs from the bundle.'
      }
    } finally {
      if (!$process.HasExited) { Stop-Process -Id $process.Id -Force }
    }
  }
} finally {
  $env:PROGRAMDATA = $previousProgramData
}
Write-Output 'GUI configuration bootstrap smoke passed'
