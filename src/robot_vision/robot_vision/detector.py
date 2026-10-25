"""
Human Victim & Object Detector module.
Supports YOLOv8 (ultralytics) with automatic seamless fallback to OpenCV HOG People Detector.
"""

import os
import cv2
import numpy as np
from typing import List, Tuple, Dict, Any, Optional

# Enforce offline mode for ultralytics so it never pings external servers
os.environ['YOLO_OFFLINE'] = '1'
os.environ['ULTRALYTICS_AUTOINSTALL'] = '0'
os.environ['ULTRALYTICS_CHECK_REQUIREMENTS'] = '0'

class Detection:
    def __init__(self, x: int, y: int, w: int, h: int, confidence: float, class_name: str = 'person'):
        self.x = int(x)
        self.y = int(y)
        self.w = int(w)
        self.h = int(h)
        self.confidence = float(confidence)
        self.class_name = class_name
        self.center_x = self.x + self.w / 2.0
        self.center_y = self.y + self.h / 2.0

class VictimDetector:
    def __init__(self, model_name: str = 'yolov8n.pt', prefer_yolo: bool = True):
        self.backend = 'none'
        self.model = None
        self.hog = None

        if prefer_yolo:
            # Resolve model file from multiple candidate local paths
            resolved_path = self._resolve_model_path(model_name)
            try:
                from ultralytics import YOLO
                self.model = YOLO(resolved_path)
                self.backend = 'yolov8'
                print(f"[VictimDetector] Initialized YOLOv8 ({resolved_path}) offline successfully.")
            except ImportError:
                print("[VictimDetector] 'ultralytics' not installed. Falling back to OpenCV HOG People Detector.")
            except Exception as e:
                print(f"[VictimDetector] Could not load YOLO model from '{resolved_path}' ({e}). Falling back to OpenCV HOG.")

        if self.backend == 'none':
            # Initialize OpenCV HOG Default People Detector (requires zero downloads)
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.backend = 'opencv_hog'
            print("[VictimDetector] Initialized OpenCV HOG People Detector backend (100% offline).")

    def _resolve_model_path(self, model_name: str) -> str:
        """Finds the local .pt model file without accessing the internet."""
        if os.path.isabs(model_name) and os.path.exists(model_name):
            return model_name

        search_dirs = [
            os.getcwd(),
            os.path.join(os.getcwd(), 'models'),
            os.path.join(os.getcwd(), 'src', 'robot_vision', 'models'),
            '/workspace',
            '/workspace/models',
            '/workspace/src/robot_vision/models',
            os.path.dirname(os.path.abspath(__file__)),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models'),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'models'),
            os.path.expanduser('~/.cache/ultralytics'),
        ]

        base_name = os.path.basename(model_name)
        for directory in search_dirs:
            candidate = os.path.join(directory, base_name)
            if os.path.isfile(candidate):
                return os.path.abspath(candidate)

        return model_name

    def detect(self, frame: np.ndarray, conf_thresh: float = 0.35) -> List[Detection]:
        """
        Run detection on a BGR frame. Returns list of Detection objects.
        """
        if frame is None or frame.size == 0:
            return []

        h, w = frame.shape[:2]
        detections: List[Detection] = []

        if self.backend == 'yolov8' and self.model is not None:
            # YOLOv8 inference
            results = self.model(frame, verbose=False, conf=conf_thresh)
            for res in results:
                boxes = res.boxes
                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    # In COCO dataset, class 0 is 'person'
                    if cls_id == 0:
                        xywh = box.xywh[0].cpu().numpy()
                        cx, cy, bw, bh = xywh
                        bx = int(cx - bw / 2.0)
                        by = int(cy - bh / 2.0)
                        conf = float(box.conf[0].item())
                        detections.append(Detection(bx, by, int(bw), int(bh), conf, 'person'))

        elif self.backend == 'opencv_hog' and self.hog is not None:
            # OpenCV HOG on downscaled frame for performance
            scale_w = 320.0
            scale_factor = scale_w / max(w, 1)
            target_h = int(h * scale_factor)
            small_frame = cv2.resize(frame, (int(scale_w), target_h))

            boxes, weights = self.hog.detectMultiScale(
                small_frame,
                winStride=(8, 8),
                padding=(4, 4),
                scale=1.05
            )

            inv_scale = 1.0 / scale_factor
            for idx, (bx, by, bw, bh) in enumerate(boxes):
                weight = float(weights[idx]) if not isinstance(weights[idx], (list, np.ndarray)) else float(weights[idx][0])
                if weight >= conf_thresh:
                    orig_x = int(bx * inv_scale)
                    orig_y = int(by * inv_scale)
                    orig_w = int(bw * inv_scale)
                    orig_h = int(bh * inv_scale)
                    detections.append(Detection(orig_x, orig_y, orig_w, orig_h, weight, 'person'))

        return detections

    def draw_overlays(self, frame: np.ndarray, detections: List[Detection],
                      distances: Optional[List[float]] = None,
                      bearings_deg: Optional[List[float]] = None) -> np.ndarray:
        """
        Draw visual bounding boxes and HUD metrics on frame.
        """
        annotated = frame.copy()
        for idx, det in enumerate(detections):
            # Draw box
            p1 = (det.x, det.y)
            p2 = (det.x + det.w, det.y + det.h)
            cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)

            # Label text
            dist_str = f" {distances[idx]:.2f}m" if distances and idx < len(distances) else ""
            bear_str = f" ({bearings_deg[idx]:+.1f}deg)" if bearings_deg and idx < len(bearings_deg) else ""
            label = f"VICTIM: {det.confidence*100:.0f}%{dist_str}{bear_str}"

            # Badge background
            cv2.rectangle(annotated, (det.x, max(0, det.y - 22)), (det.x + len(label) * 9, max(22, det.y)), (0, 200, 0), -1)
            cv2.putText(annotated, label, (det.x + 3, max(16, det.y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

        return annotated
