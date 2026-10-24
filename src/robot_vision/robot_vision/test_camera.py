#!/usr/bin/env python3
"""
Standalone Camera & Perception Diagnostic Tool for ASUS GX10.
Tests:
1. Logitech webcam via /dev/video0 or network stream (e.g. http://<pi_ip>:5000/video).
2. Measures FPS and frame latency.
3. Tests YOLOv8 / HOG victim detection and bounding boxes.
4. Floor ROI edge density computation.
"""

import sys
import time
import cv2
import numpy as np

from robot_vision.detector import VictimDetector

def main():
    source = sys.argv[1] if len(sys.argv) > 1 else "0"
    print(f"============================================================")
    print(f"  Perception Diagnostic Tool")
    print(f"  Target Source: '{source}'")
    print(f"============================================================")

    # Open video capture
    if source.isdigit():
        cap = cv2.VideoCapture(int(source), cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(int(source))
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    else:
        cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera source '{source}'.")
        print("Check cable connection, permissions, or network URL.")
        return 1

    detector = VictimDetector(prefer_yolo=True)
    print(f"[INFO] Using detector backend: {detector.backend}")

    prev_time = time.time()
    frame_count = 0
    fps = 0.0

    print("[INFO] Starting test loop. Press 'q' or Ctrl+C to exit.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[WARN] Failed to read frame from camera.")
                time.sleep(0.05)
                continue

            frame_count += 1
            curr_time = time.time()
            if curr_time - prev_time >= 1.0:
                fps = frame_count / (curr_time - prev_time)
                frame_count = 0
                prev_time = curr_time

            h, w = frame.shape[:2]

            # 1. Floor ROI Edge Detection
            roi_start = int(h * 0.60)
            roi = frame[roi_start:h, 0:w]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            density = float(np.count_nonzero(edges)) / float(max(1, edges.size))

            # 2. Victim Detection
            detections = detector.detect(frame, conf_thresh=0.35)
            annotated = detector.draw_overlays(frame, detections)

            # Draw HUD
            hud_text = f"FPS: {fps:.1f} | Victims: {len(detections)} | Floor Edge Den: {density*100:.1f}%"
            cv2.rectangle(annotated, (0, 0), (w, 35), (20, 20, 20), -1)
            cv2.putText(annotated, hud_text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Draw Floor ROI outline
            cv2.rectangle(annotated, (0, roi_start), (w, h), (255, 0, 0), 2)

            cv2.imshow("Perception Diagnostic", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\n[INFO] Diagnostic completed cleanly.")

    return 0

if __name__ == '__main__':
    sys.exit(main())
