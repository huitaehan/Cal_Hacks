# ==============================================================================
# Fast Sync Script for Raspberry Pi (Robot Host)
# ==============================================================================
$PI_IP = "192.168.0.10"

Write-Host "`n>>> Syncing nodes to Raspberry Pi ($PI_IP)... (Enter password for robot1 if prompted)" -ForegroundColor Cyan
scp c:\Users\Huitae\Cal_Hacks\motor_node.py c:\Users\Huitae\Cal_Hacks\pi_bringup.py c:\Users\Huitae\Cal_Hacks\src\robot_vision\robot_vision\pi_cam_node.py c:\Users\Huitae\Cal_Hacks\src\robot_vision\robot_vision\ultrasonic_node.py robot1@${PI_IP}:~/

Write-Host "`n>>> [SUCCESS] Raspberry Pi updated!" -ForegroundColor Green
Write-Host "Run on Raspberry Pi:" -ForegroundColor Yellow
Write-Host "  Sensors: python3 ~/pi_bringup.py" -ForegroundColor Yellow
Write-Host "  Motors:  python3 ~/motor_node.py`n" -ForegroundColor Yellow
