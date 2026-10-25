# ==============================================================================
# One-Click Fast Sync Script for Cal Hacks Autonomous Robot
# Deploys code to ASUS GX10 and Raspberry Pi over local LAN (offline-friendly)
# ==============================================================================

$GX10_IP = "192.168.0.232"
$PI_IP = "192.168.0.10"

Write-Host "`n>>> [1/2] Syncing to ASUS GX10 ($GX10_IP)..." -ForegroundColor Cyan
scp c:\Users\Huitae\Cal_Hacks\run_gx10.sh asus@${GX10_IP}:/home/asus/calhacks/run_gx10.sh
scp c:\Users\Huitae\Cal_Hacks\launch_gx10.py asus@${GX10_IP}:/home/asus/calhacks/launch_gx10.py
scp c:\Users\Huitae\Cal_Hacks\src\robot_vision\robot_vision\visual_wall_detector.py asus@${GX10_IP}:/home/asus/calhacks/src/robot_vision/robot_vision/visual_wall_detector.py
scp c:\Users\Huitae\Cal_Hacks\src\robot_core\robot_core\navigator_node.py asus@${GX10_IP}:/home/asus/calhacks/src/robot_core/robot_core/navigator_node.py

Write-Host "`n>>> [2/2] Syncing to Raspberry Pi ($PI_IP)..." -ForegroundColor Cyan
scp c:\Users\Huitae\Cal_Hacks\pi_bringup.py robot1@${PI_IP}:~/pi_bringup.py
scp c:\Users\Huitae\Cal_Hacks\src\robot_vision\robot_vision\pi_cam_node.py robot1@${PI_IP}:~/pi_cam_node.py
scp c:\Users\Huitae\Cal_Hacks\src\robot_vision\robot_vision\ultrasonic_node.py robot1@${PI_IP}:~/ultrasonic_node.py

Write-Host "`n>>> [SUCCESS] All nodes synchronized successfully across LAN!" -ForegroundColor Green
Write-Host "To run robot:"
Write-Host "  Raspberry Pi: python3 ~/pi_bringup.py" -ForegroundColor Yellow
Write-Host "  ASUS GX10:    python3 launch_gx10.py" -ForegroundColor Yellow
