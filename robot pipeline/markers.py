"""Write the printable markers: markers/print.html, one marker per page at a set size.

    python markers.py            # floor markers 18 cm, robot markers 12 cm
    python markers.py --robot-cm 9
"""
import argparse
import base64
import json
from pathlib import Path

import cv2

from tracker import CONFIG, DICT

OUT = Path(__file__).parent / "markers"


def png_b64(marker_id: int) -> str:
    img = cv2.aruco.generateImageMarker(DICT, marker_id, 600)
    ok, buf = cv2.imencode(".png", img)
    assert ok
    return base64.b64encode(buf.tobytes()).decode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--floor-cm", type=float, default=18.0)
    ap.add_argument("--robot-cm", type=float, default=12.0)
    args = ap.parse_args()
    cfg = json.loads(CONFIG.read_text())
    pages = []
    for mid, (x, y) in cfg["floor_markers"].items():
        pages.append((int(mid), args.floor_cm, f"FLOOR marker {mid}",
                      f"Tape flat on the floor, centre exactly on ({x}, {y}) m. Any rotation is fine."))
    for name, r in cfg["robots"].items():
        pages.append((r["marker"], args.robot_cm, f"ROBOT {name} (marker {r['marker']})",
                      "Tape flat on top of the robot, centred, arrow toward the robot's front. Keep it flat and uncovered."))
    body = []
    for mid, cm, title, note in pages:
        arrow = '<div class="arrow">&#9650; FRONT</div>' if title.startswith("ROBOT") else ""
        body.append(f'<section><h1>{title}</h1>{arrow}'
                    f'<img src="data:image/png;base64,{png_b64(mid)}" style="width:{cm}cm;height:{cm}cm">'
                    f'<p>{note}</p><p class="s">Print at 100 % scale ("actual size"). Leave the white margin.</p></section>')
    OUT.mkdir(exist_ok=True)
    (OUT / "print.html").write_text(
        "<!doctype html><meta charset=utf-8><title>Robot markers</title><style>"
        "body{font-family:sans-serif;margin:0} section{page-break-after:always;text-align:center;padding:1cm}"
        "img{image-rendering:pixelated;border:1.5cm solid #fff} h1{font-size:20pt;margin:0 0 .3cm}"
        ".arrow{font-size:28pt;font-weight:bold} p{font-size:12pt} .s{color:#666;font-size:10pt}"
        "</style>" + "".join(body), encoding="utf-8")
    print(f"wrote {OUT / 'print.html'} ({len(pages)} pages)")


if __name__ == "__main__":
    main()
