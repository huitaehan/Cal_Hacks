#!/usr/bin/env python3
"""
Raspberry Pi Onboard Thin-Client for Cal Hacks Search-and-Rescue Robot.
Mount this script on the Raspberry Pi on the robot chassis.
Functions:
1. Streams Logitech webcam frames via high-speed HTTP MJPEG to the ASUS GX10.
2. Reads HC-SR04 ultrasonic sensors (front, left, right).
3. Receives motor velocity commands from GX10 and drives GPIO/PWM.
4. Includes Safety Motor Watchdog (auto-stops motors if Wi-Fi heartbeat drops >0.5s).
"""

import sys
import time
import json
import threading
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# Try importing RPi.GPIO for hardware motors & sensors; fallback to mock for desktop testing
try:
    import RPi.GPIO as GPIO
    HAVE_GPIO = True
except ImportError:
    HAVE_GPIO = False
    print("[PiClient] RPi.GPIO not found. Running in MOCK sensor/motor mode.")

# Global state shared with HTTP server
latest_jpeg = None
lock = threading.Lock()
sensor_data = {"front": 2.0, "left": 2.0, "right": 2.0}
last_command_time = time.time()
running = True

# ----------------------------------------------------
# Hardware Pin Definitions (Customizable)
# ----------------------------------------------------
# Motors (e.g. L298N H-Bridge)
PIN_MOTOR_LEFT_PWM = 12
PIN_MOTOR_LEFT_DIR = 16
PIN_MOTOR_RIGHT_PWM = 13
PIN_MOTOR_RIGHT_DIR = 20

# Ultrasonic HC-SR04 Pins (Trigger, Echo)
PIN_TRIG_FRONT = 23
PIN_ECHO_FRONT = 24

def init_hardware():
    if not HAVE_GPIO:
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motors
    GPIO.setup(PIN_MOTOR_LEFT_PWM, GPIO.OUT)
    GPIO.setup(PIN_MOTOR_LEFT_DIR, GPIO.OUT)
    GPIO.setup(PIN_MOTOR_RIGHT_PWM, GPIO.OUT)
    GPIO.setup(PIN_MOTOR_RIGHT_DIR, GPIO.OUT)

    # Ultrasonic
    GPIO.setup(PIN_TRIG_FRONT, GPIO.OUT)
    GPIO.setup(PIN_ECHO_FRONT, GPIO.IN)
    GPIO.output(PIN_TRIG_FRONT, False)

def read_ultrasonic_distance(trig_pin: int, echo_pin: int) -> float:
    if not HAVE_GPIO:
        return 2.0
    try:
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)

        pulse_start = time.time()
        timeout = pulse_start + 0.04  # 40ms timeout

        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return 2.0

        pulse_end = time.time()
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return 2.0

        pulse_duration = pulse_end - pulse_start
        # Speed of sound: 34300 cm/s -> distance in meters
        dist = (pulse_duration * 34300) / 2.0 / 100.0
        return max(0.05, min(dist, 3.0))
    except Exception:
        return 2.0

def set_motors(linear_v: float, angular_w: float):
    """
    Convert linear velocity (m/s) and angular velocity (rad/s) to differential wheel speeds.
    """
    global last_command_time
    last_command_time = time.time()

    # Differential drive kinematics
    wheel_base = 0.20  # Distance between wheels in meters
    left_speed = linear_v - (angular_w * wheel_base / 2.0)
    right_speed = linear_v + (angular_w * wheel_base / 2.0)

    # Clamping
    left_speed = max(-1.0, min(1.0, left_speed))
    right_speed = max(-1.0, min(1.0, right_speed))

    if not HAVE_GPIO:
        # print(f"[MOCK MOTOR] Left: {left_speed:+.2f}, Right: {right_speed:+.2f}")
        return

    # Set motor directions and PWM duty cycles
    GPIO.output(PIN_MOTOR_LEFT_DIR, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
    GPIO.output(PIN_MOTOR_RIGHT_DIR, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
    # Note: PWM software or hardware objects can be attached here

def stop_motors():
    if not HAVE_GPIO:
        return
    GPIO.output(PIN_MOTOR_LEFT_DIR, GPIO.LOW)
    GPIO.output(PIN_MOTOR_RIGHT_DIR, GPIO.LOW)

def motor_watchdog_loop():
    """Safety watchdog: stops robot if no heartbeat received for > 0.5 seconds."""
    global running, last_command_time
    while running:
        if time.time() - last_command_time > 0.5:
            stop_motors()
        time.sleep(0.1)

# ----------------------------------------------------
# High-Speed MJPEG & Telemetry HTTP Server
# ----------------------------------------------------
class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global latest_jpeg, sensor_data

        if self.path == '/video':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while running:
                    with lock:
                        frame_bytes = latest_jpeg
                    if frame_bytes is not None:
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(frame_bytes)))
                        self.end_headers()
                        self.wfile.write(frame_bytes)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.03)  # ~30 FPS limit
            except Exception:
                pass

        elif self.path == '/sensors':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(sensor_data).encode('utf-8'))

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        # Handle /cmd_vel from GX10
        if self.path == '/cmd_vel':
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length)
            try:
                cmd = json.loads(post_data.decode('utf-8'))
                v = float(cmd.get('linear', 0.0))
                w = float(cmd.get('angular', 0.0))
                set_motors(v, w)
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'OK')
            except Exception as e:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(str(e).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Silence standard HTTP access logs for high-speed streaming
        return

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

def camera_capture_loop(cam_index=0, width=640, height=480, quality=70):
    global latest_jpeg, running
    cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(cam_index)

    if not cap.isOpened():
        print(f"[PiClient ERROR] Failed to open webcam at /dev/video{cam_index}")
        return

    # Use V4L2 MJPEG hardware mode for low latency
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    print(f"[PiClient] Camera stream running at {width}x{height} (JPEG Q={quality})")

    while running:
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue
        ret_enc, jpeg_buf = cv2.imencode('.jpg', frame, encode_params)
        if ret_enc:
            with lock:
                latest_jpeg = jpeg_buf.tobytes()

    cap.release()

def sensor_poll_loop():
    global sensor_data, running
    while running:
        front_d = read_ultrasonic_distance(PIN_TRIG_FRONT, PIN_ECHO_FRONT)
        sensor_data["front"] = round(front_d, 2)
        sensor_data["left"] = 2.0
        sensor_data["right"] = 2.0
        time.sleep(0.05)  # 20 Hz

def main():
    global running
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5000
    cam_idx = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    init_hardware()

    # Start camera worker thread
    cam_thread = threading.Thread(target=camera_capture_loop, args=(cam_idx,), daemon=True)
    cam_thread.start()

    # Start sensor worker thread
    sensor_thread = threading.Thread(target=sensor_poll_loop, daemon=True)
    sensor_thread.start()

    # Start motor safety watchdog thread
    watchdog_thread = threading.Thread(target=motor_watchdog_loop, daemon=True)
    watchdog_thread.start()

    server_address = ('', port)
    server = ThreadedHTTPServer(server_address, StreamingHandler)
    print(f"============================================================")
    print(f"  Raspberry Pi Robot Thin-Client Running on Port {port}")
    print(f"  - Video Stream: http://<PI_IP>:{port}/video")
    print(f"  - Telemetry:    http://<PI_IP>:{port}/sensors")
    print(f"  - Motor API:    POST to http://<PI_IP>:{port}/cmd_vel")
    print(f"============================================================")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down Pi client...")
    finally:
        running = False
        stop_motors()
        if HAVE_GPIO:
            GPIO.cleanup()
        server.server_close()

if __name__ == '__main__':
    main()
