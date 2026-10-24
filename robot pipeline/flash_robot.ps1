# Build and flash the robot motor firmware (edit firmware\robot_motor\wifi_config.h first: network name and password).
#   .\flash_robot.ps1 -Port COM12 -Robot A
#   .\flash_robot.ps1 -Port COM13 -Robot B
param(
    [Parameter(Mandatory)][string]$Port,
    [Parameter(Mandatory)][ValidateSet("A", "B")][string]$Robot,
    [string]$Fqbn = "esp32:esp32:esp32c5"
)
$cli = Join-Path $env:LOCALAPPDATA "Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
# Build in a short path: this folder's path has spaces and an apostrophe.
$work = "C:\esp\work\robot_motor"
Remove-Item -Recurse -Force $work -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $work | Out-Null
Copy-Item (Join-Path $PSScriptRoot "firmware\robot_motor\*") $work
$cfg = Join-Path $work "wifi_config.h"
(Get-Content $cfg) -replace '#define ROBOT_ID\s+"[^"]*"', "#define ROBOT_ID  `"$Robot`"" | Set-Content $cfg
& $cli compile --fqbn $Fqbn $work
if ($LASTEXITCODE) { exit 1 }
& $cli upload --fqbn $Fqbn -p $Port $work
