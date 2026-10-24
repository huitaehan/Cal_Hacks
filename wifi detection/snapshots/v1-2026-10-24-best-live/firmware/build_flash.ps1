# Build the ESP32-C5 presence node firmware and (optionally) flash it.
#
#   .\build_flash.ps1                       # build only
#   .\build_flash.ps1 -Ports COM10,COM11    # build + flash these boards
#   .\build_flash.ps1 -All                  # build + flash every connected ESP32-C5
#
# -All checks each Espressif USB device first and skips any board that is not
# an ESP32-C5 or whose port is busy (e.g. held by the dashboard server).
#
# ESP-IDF cannot build from a path containing spaces, so the project is
# mirrored to $WorkDir and built there.
param(
    [string[]]$Ports = @(),
    [switch]$All,
    [string]$IdfPath = "C:\esp\esp-idf",
    [string]$WorkDir = "C:\esp\work\c5_presence_node"
)
# Native tools write progress to stderr; rely on exit codes instead.
$ErrorActionPreference = "Continue"
$src = Join-Path $PSScriptRoot "c5_presence_node"
$py = Join-Path $PSScriptRoot "..\.venv\Scripts\python.exe"

robocopy $src $WorkDir /MIR /XD build managed_components /XF sdkconfig sdkconfig.old /NFL /NDL /NJH /NJS /NP | Out-Null
if ($LASTEXITCODE -ge 8) { throw "robocopy failed ($LASTEXITCODE)" }

$cmd = "call `"$IdfPath\export.bat`" >nul && cd /d `"$WorkDir`" && idf.py set-target esp32c5 build"
if (-not (Test-Path "$WorkDir\build\CMakeCache.txt")) {
    cmd /c $cmd
} else {
    cmd /c "call `"$IdfPath\export.bat`" >nul && cd /d `"$WorkDir`" && idf.py build"
}
if ($LASTEXITCODE -ne 0) { throw "build failed" }

if ($All) {
    $found = Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -match 'VID_303A' } |
        ForEach-Object { if ($_.Name -match '\((COM\d+)\)') { $Matches[1] } } | Sort-Object -Unique
    $Ports = @()
    foreach ($p in $found) {
        $out = & $py -m esptool --port $p chip-id 2>&1 | Out-String
        if ($out -notmatch 'Detecting chip type\.\.\. (\S+)') {
            Write-Warning "$p skipped: could not talk to the board (port in use by the dashboard server?)"
        } elseif ($Matches[1] -ne 'ESP32-C5') {
            Write-Warning "$p skipped: it is an $($Matches[1]), this firmware is for ESP32-C5"
        } else {
            $Ports += $p
        }
    }
    if (-not $Ports) { Write-Host "No ESP32-C5 boards to flash."; return }
    Write-Host "ESP32-C5 boards to flash: $($Ports -join ', ')"
}

$flashed = @()
foreach ($p in $Ports) {
    Write-Host "== Flashing $p"
    cmd /c "call `"$IdfPath\export.bat`" >nul && cd /d `"$WorkDir`" && idf.py -p $p flash"
    if ($LASTEXITCODE -ne 0) { throw "flash failed on $p" }
    $flashed += $p
}
if ($flashed) { Write-Host "Flashed: $($flashed -join ', ')" }
