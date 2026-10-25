# ==============================================================================
# Fast Sync Script for ASUS GX10 (AI Base Station)
# ==============================================================================
$GX10_IP = "192.168.0.232"

Write-Host "`n>>> Syncing nodes to ASUS GX10 ($GX10_IP)... (Enter password for asus if prompted)" -ForegroundColor Cyan
scp c:\Users\Huitae\Cal_Hacks\run_gx10.sh c:\Users\Huitae\Cal_Hacks\launch_gx10.py asus@${GX10_IP}:/home/asus/calhacks/
scp c:\Users\Huitae\Cal_Hacks\src\robot_vision\robot_vision\visual_wall_detector.py asus@${GX10_IP}:/home/asus/calhacks/src/robot_vision/robot_vision/visual_wall_detector.py
scp c:\Users\Huitae\Cal_Hacks\src\robot_core\robot_core\navigator_node.py asus@${GX10_IP}:/home/asus/calhacks/src/robot_core/robot_core/navigator_node.py

Write-Host "`n>>> [SUCCESS] ASUS GX10 updated!" -ForegroundColor Green
Write-Host "Run on GX10: /home/asus/calhacks/run_gx10.sh and then python3 launch_gx10.py`n" -ForegroundColor Yellow
