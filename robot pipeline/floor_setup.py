"""Map the camera onto the robots' tops (colour tracking): click a robot standing on each area corner.

    python floor_setup.py [--camera 1]

Tape the 4 corners of the search area on the floor. Stand a robot on a corner mark, then click its blue
centre (the click snaps to the nearest blue blob), and repeat for each corner, in this order: front-left,
front-right, back-right, back-left (the "area" corners in config.json; front = the door side).
Clicking at robot-top height matters: the colours sit on the robots' tops, and a mapping clicked on the
floor would put robots far from the camera tens of centimetres off. Run colours.py first so blue is tuned.
Keys: u undo, s save floor.json, q quit. Redo this whenever the camera moves.
"""
import argparse
import json

import cv2
import numpy as np

from colour_tracker import DEFAULT_COLOURS, ColourTracker
from tracker import COLOURS, CONFIG, FLOOR, apply_h, open_camera

SNAP_PX = 40        # a click this close to a blue blob uses the blob's centre


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int)
    args = ap.parse_args()
    cfg = json.loads(CONFIG.read_text())
    if args.camera is not None:
        cfg["camera"]["index"] = args.camera
    (x0, y0), (x1, y1) = cfg["area"]
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    names = ["front-left", "front-right", "back-right", "back-left"]
    clicks = []
    colours = json.loads(COLOURS.read_text()) if COLOURS.exists() else DEFAULT_COLOURS
    blue = ColourTracker({}, colours=colours)        # no dot colours: only the blue centres are found
    blues = []

    def on_click(ev, x, y, *_):
        if ev != cv2.EVENT_LBUTTONDOWN or len(clicks) >= 4:
            return
        near = [b for b in blues if (b[0] - x) ** 2 + (b[1] - y) ** 2 <= SNAP_PX ** 2]
        clicks.append(min(near, key=lambda b: (b[0] - x) ** 2 + (b[1] - y) ** 2) if near else (x, y))
        print(f"{names[len(clicks) - 1]}: {'robot centre' if near else 'clicked point (no blue blob near)'} "
              f"at pixel ({clicks[-1][0]:.0f}, {clicks[-1][1]:.0f})")

    cap = open_camera(cfg["camera"])
    cv2.namedWindow("floor setup", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("floor setup", on_click)
    H = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            blobs = blue.detect(frame)
            blues[:] = [(cx, cy) for col, cx, cy, _ in blobs if col == "blue"]
            blue.annotate(frame, blobs)
            for (px, py), n in zip(clicks, names):
                cv2.circle(frame, (int(px), int(py)), 8, (0, 0, 255), -1)
                cv2.putText(frame, n, (int(px) + 10, int(py) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            if len(clicks) == 4:
                H = cv2.getPerspectiveTransform(np.float32(clicks), np.float32(corners))
                Hi = np.linalg.inv(H)
                for gx in np.arange(x0, x1 + 1e-9, 0.5):       # 0.5 m grid to check the mapping by eye
                    a, b = apply_h(Hi, [(gx, y0), (gx, y1)]).astype(int)
                    cv2.line(frame, tuple(a), tuple(b), (0, 200, 0), 1)
                for gy in np.arange(y0, y1 + 1e-9, 0.5):
                    a, b = apply_h(Hi, [(x0, gy), (x1, gy)]).astype(int)
                    cv2.line(frame, tuple(a), tuple(b), (0, 200, 0), 1)
                msg = "0.5 m grid at robot-top height: it should roughly match the floor.  s: save   u: undo   q: quit"
            else:
                msg = (f"robot on the {names[len(clicks)]} corner {corners[len(clicks)]}: click its blue centre"
                       "   u: undo   q: quit")
            cv2.putText(frame, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(frame, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow("floor setup", frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            if key == ord("u") and clicks:
                clicks.pop()
                H = None
            if key == ord("s") and H is not None:
                FLOOR.write_text(json.dumps({"H": H.tolist(), "clicked_px": [list(map(float, c)) for c in clicks],
                                             "floor": corners}, indent=2))
                print(f"saved {FLOOR.name}")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
