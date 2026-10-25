"""The Pi stand-in for the robot firmware: same wheel maths, same UDP commands, same 300 ms stop."""
import threading
import time

from drive import RobotLink
from pi_robot import Robot, serve, wheel_speeds


class FakePWM:
    def __init__(self):
        self.value = 0.0


def make_robot(watchdog_s=0.3):
    pins = [FakePWM() for _ in range(6)]
    return Robot(pins, watchdog_s), pins


def test_wheel_maths_match_the_firmware():
    # dir 0 at speed 60: firmware F1 = 0.578*60, F2 = 0, F3 = -0.578*60, scaled so the fastest wheel is 60.
    # move() sends F1 -> A, F3 -> B, F2 -> C.
    assert wheel_speeds(60, 0, 0) == (60, -60, 0)
    # dir 90: F1 = F3 = -0.333*v, F2 = 0.567*v; F2 is the fastest -> motor C at full speed.
    a, b, c = wheel_speeds(50, 90, 0)
    assert c == 50 and a == b and -30 < a < -28, (a, b, c)
    # Pure turn: every wheel the same way (13.32 * 40 / 13.32 = 39.999..., truncated by (int) as on the ESP).
    # Out-of-range inputs are clamped like constrain().
    assert wheel_speeds(0, 0, 40) == (39, 39, 39)
    assert wheel_speeds(250, 0, 0) == (100, -100, 0)


def test_pwm_pairs_follow_the_esp_pin_pairs():
    robot, pins = make_robot()
    robot.handle("D 1 60 0 0", 0.0)
    # Motor A forward (ESP pin 2), motor B backward (ESP pin 1), motor C off.
    assert abs(pins[0].value - 153 / 255) < 1e-9 and pins[1].value == 0
    assert pins[2].value == 0 and abs(pins[3].value - 153 / 255) < 1e-9
    assert pins[4].value == 0 and pins[5].value == 0
    robot.handle("S", 0.1)
    assert all(p.value == 0 for p in pins)


def test_motors_stop_without_commands_and_on_nan():
    robot, pins = make_robot()
    robot.handle("D 1 50 0 0", 10.0)
    robot.tick(10.2)
    assert any(p.value for p in pins)
    robot.tick(10.31)
    assert not any(p.value for p in pins)
    robot.handle("D 2 50 0 0", 11.0)
    robot.handle("D 3 nan 0 0", 11.05)
    assert not any(p.value for p in pins)
    robot.handle("D 4 garbage", 11.1)               # malformed: ignored, still stopped
    assert not any(p.value for p in pins)


def test_drive_py_commands_reach_the_pi_over_udp():
    robot, pins = make_robot(watchdog_s=5.0)
    stop, bound = threading.Event(), threading.Event()
    port = 47210
    t = threading.Thread(target=serve, args=(robot, "A", port, 0, stop, bound), daemon=True)
    t.start()
    assert bound.wait(2)
    link = RobotLink(cmd_port=port, hello_port=0, static_ips={"A": "127.0.0.1"})
    try:
        link.drive("A", 60, 0)
        end = time.time() + 2
        while not pins[0].value and time.time() < end:
            time.sleep(0.01)
        assert abs(pins[0].value - 153 / 255) < 1e-9, [p.value for p in pins]
        link.stop("A")
        end = time.time() + 2
        while pins[0].value and time.time() < end:
            time.sleep(0.01)
        assert not any(p.value for p in pins)
    finally:
        stop.set()
        t.join(2)
        link.close()


if __name__ == "__main__":
    test_wheel_maths_match_the_firmware()
    test_pwm_pairs_follow_the_esp_pin_pairs()
    test_motors_stop_without_commands_and_on_nan()
    test_drive_py_commands_reach_the_pi_over_udp()
    print("ok")
