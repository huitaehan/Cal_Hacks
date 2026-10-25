#!/bin/bash
# Installs the robot motor controller (pi_robot.py) on a Raspberry Pi as a service that starts at boot.
#   sudo bash install.sh A        (robot A; B for robot B)
# Run it from wherever copy_to_pi.ps1 put the files: /boot/firmware/robot on the SD card, or ~/robot over SSH.
set -e
ROBOT="${1:-A}"
HERE="$(cd "$(dirname "$0")" && pwd)"
case "$ROBOT" in A|B) ;; *) echo "usage: sudo bash install.sh A|B"; exit 1 ;; esac
[ "$(id -u)" = 0 ] || { echo "run it with sudo: sudo bash $0 $ROBOT"; exit 1; }

check_pwm() { python3 -c "from gpiozero import PWMOutputDevice; d = PWMOutputDevice(17, frequency=1000); d.close()"; }
if ! check_pwm 2>/dev/null; then
    echo "installing gpiozero and lgpio (needs internet through the hotspot)"
    apt-get update -q && apt-get install -y -q python3-gpiozero python3-lgpio
    check_pwm
fi

install -d /opt/robot
install -m 644 "$HERE/pi_robot.py" /opt/robot/pi_robot.py
sed "s/@ROBOT@/$ROBOT/g" "$HERE/pi-robot.service" > /etc/systemd/system/pi-robot.service

# WiFi power saving holds packets back for 100 ms or more, and the robot stops after 300 ms without
# commands (the ESP32 firmware turned it off for the same reason).
install -d /etc/NetworkManager/conf.d
printf '[connection]\nwifi.powersave = 2\n' > /etc/NetworkManager/conf.d/robot-wifi-powersave-off.conf
iw dev wlan0 set power_save off 2>/dev/null || true

systemctl daemon-reload
systemctl enable pi-robot
systemctl restart pi-robot
sleep 3
systemctl --no-pager --lines=5 status pi-robot || true
echo
echo "Robot $ROBOT is running and starts by itself at every boot."
echo "  logs:          journalctl -u pi-robot -f"
echo "  wiring check:  sudo systemctl stop pi-robot; sudo python3 /opt/robot/pi_robot.py $ROBOT --spin; sudo systemctl start pi-robot"
