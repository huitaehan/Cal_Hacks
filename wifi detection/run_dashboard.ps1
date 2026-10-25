# Start the WiFi sensing server and open the 3D mission view.
#   .\run_dashboard.ps1                 # this laptop only
#   .\run_dashboard.ps1 -Network        # also reachable by the robot team's laptop (prints the URL)
#   .\run_dashboard.ps1 -Record         # also save raw CSI under host\recordings
#   .\run_dashboard.ps1 -Detector espectre   # the previous detector (ESPectre, two agreeing pairs)
param([switch]$Record, [switch]$Network, [int]$Port = 8000, [string]$Detector = "wander")

$py = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
$server = Join-Path $PSScriptRoot "host\presence_server.py"
$serverArgs = @($server, "--http-port", $Port, "--detector", $Detector)
if ($Record) { $serverArgs += "--record" }
if ($Network) { $serverArgs += @("--host", "0.0.0.0") }

Start-Job -ScriptBlock { param($u) Start-Sleep 3; Start-Process $u } -ArgumentList "http://localhost:$Port/3d" | Out-Null
& $py @serverArgs
