# Put the robot motor controller on a Raspberry Pi: pi_robot.py, pi\install.sh and pi\pi-robot.service.
#   .\copy_to_pi.ps1                          # onto the freshly written SD card (its boot partition, found by itself)
#   .\copy_to_pi.ps1 -Drive E:                # same, naming the SD card's drive
#   .\copy_to_pi.ps1 -Ssh robot@robota.local  # onto a running Pi over WiFi, and install it there (robot A)
# From the SD card, install once on the Pi with:  sudo bash /boot/firmware/robot/install.sh A
param(
    [string]$Drive = "",
    [string]$Ssh = "",
    [ValidateSet("A", "B")][string]$Robot = "A"
)
$ErrorActionPreference = "Stop"
$files = @{
    "pi_robot.py"      = Join-Path $PSScriptRoot "pi_robot.py"
    "install.sh"       = Join-Path $PSScriptRoot "pi\install.sh"
    "pi-robot.service" = Join-Path $PSScriptRoot "pi\pi-robot.service"
}

function Write-LF($src, $dst) {
    # Linux line endings: bash can't run a script with Windows ones.
    $text = [IO.File]::ReadAllText($src) -replace "`r`n", "`n"
    [IO.File]::WriteAllText($dst, $text, (New-Object Text.UTF8Encoding $false))
}

if ($Ssh) {
    $stage = Join-Path $env:TEMP "pi_robot_stage\robot"
    Remove-Item -Recurse -Force (Split-Path $stage) -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Force $stage | Out-Null
    foreach ($name in $files.Keys) { Write-LF $files[$name] (Join-Path $stage $name) }
    Write-Host "Copying to ${Ssh}:robot (it may ask for the Pi's password)"
    scp -r $stage "${Ssh}:"
    if ($LASTEXITCODE) { throw "copy failed: is the Pi on the hotspot, and is the name right?" }
    Write-Host "Installing robot $Robot (it may ask for the password again)"
    ssh -t $Ssh "sudo bash robot/install.sh $Robot"
    exit $LASTEXITCODE
}

if (-not $Drive) {
    $found = @(Get-PSDrive -PSProvider FileSystem | Where-Object {
        (Test-Path (Join-Path $_.Root "cmdline.txt")) -and (Test-Path (Join-Path $_.Root "config.txt")) })
    if ($found.Count -ne 1) {
        throw "found $($found.Count) Raspberry Pi SD cards; name one with -Drive E: (the drive with cmdline.txt and config.txt)"
    }
    $Drive = $found[0].Root
}
$root = (Resolve-Path $Drive).Path
if (-not ((Test-Path (Join-Path $root "cmdline.txt")) -and (Test-Path (Join-Path $root "config.txt")))) {
    throw "$root has no cmdline.txt and config.txt: that's not a Raspberry Pi SD card's boot partition"
}
$dest = Join-Path $root "robot"
New-Item -ItemType Directory -Force $dest | Out-Null
foreach ($name in $files.Keys) { Write-LF $files[$name] (Join-Path $dest $name) }
Write-Host "Copied to $dest. On the Pi, run once:  sudo bash /boot/firmware/robot/install.sh $Robot"
