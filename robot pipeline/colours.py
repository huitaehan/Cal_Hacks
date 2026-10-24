"""Tune the robot colours to the room's lighting by clicking them.

    python colours.py [--camera 1]

Press b, p or g to pick a colour (blue centre, pink dot, green dot), then click that colour on a robot,
a few times and in different parts of the area: each click widens the range to include it.
Circles show what's detected with the current ranges; each robot should show 1 blue and 4 dots, and
nothing else in the room should light up.
Keys: m show the current colour's mask, r forget the current colour's clicks, s save colours.json, q quit.
"""
import argparse
import json

import cv2
import numpy as np

from colour_tracker import DEFAULT_COLOURS, ColourTracker, colour_mask, range_from_samples
from tracker import COLOURS, CONFIG, open_camera

KEYS = {ord("b"): "blue", ord("p"): "pink", ord("g"): "green"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int)
    args = ap.parse_args()
    cfg = json.loads(CONFIG.read_text())
    if args.camera is not None:
        cfg["camera"]["index"] = args.camera
    ranges = json.loads(COLOURS.read_text()) if COLOURS.exists() else dict(DEFAULT_COLOURS)
    tr = ColourTracker({n: r["pattern"] for n, r in cfg["robots"].items()}, H=None, colours=ranges)
    samples = {c: [] for c in ranges}
    state = {"colour": "blue", "mask": False, "hsv": None}

    def on_click(ev, x, y, *_):
        if ev != cv2.EVENT_LBUTTONDOWN or state["hsv"] is None:
            return
        patch = state["hsv"][max(0, y - 2):y + 3, max(0, x - 2):x + 3].reshape(-1, 3)
        col = state["colour"]
        samples[col].append(tuple(int(v) for v in np.median(patch, axis=0)))
        ranges[col] = range_from_samples(samples[col])
        print(f"{col}: sample {samples[col][-1]} -> range {ranges[col]}")

    cap = open_camera(cfg["camera"])
    cv2.namedWindow("colours", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("colours", on_click)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            state["hsv"] = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
            blobs = tr.detect(frame)
            if state["mask"]:
                view = cv2.cvtColor(colour_mask(state["hsv"], ranges[state["colour"]]), cv2.COLOR_GRAY2BGR)
            else:
                view = frame
                tr.annotate(view, blobs)
            counts = {c: sum(1 for b in blobs if b[0] == c) for c in ranges}
            msg = (f"clicking: {state['colour'].upper()} ({len(samples[state['colour']])} clicks)   blobs: "
                   + ", ".join(f"{c} {n}" for c, n in counts.items())
                   + "   b/p/g colour  m mask  r reset  s save  q quit")
            cv2.putText(view, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(view, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("colours", view)
            key = cv2.waitKey(30) & 0xFF
            if key in KEYS:
                state["colour"] = KEYS[key]
            elif key == ord("m"):
                state["mask"] = not state["mask"]
            elif key == ord("r"):
                samples[state["colour"]] = []
                ranges[state["colour"]] = DEFAULT_COLOURS[state["colour"]]
            elif key == ord("s"):
                COLOURS.write_text(json.dumps(ranges, indent=2))
                print(f"saved {COLOURS.name}")
            elif key == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
