"""Move the WiFi boards in the floor plan to a new area size.

    python wifi_area.py 197 238                  # width (left-right) and depth (front-back) in cm
    python wifi_area.py 197 238 --out 0          # boards right on the tape corners
    python wifi_area.py 197 238 --layout edges   # boards at the middle of each edge instead of the corners

Each board keeps its own corner (or edge middle) and, unless --out is given, its distance out from the tape.
The WiFi server reloads the floor plan by itself; press Recalibrate on the dashboard after moving the boards.
setup_camera.py does the same when it saves a new zone size.
"""
import argparse
import copy
import json
import math
from pathlib import Path

from tracker import CONFIG

HERE = Path(__file__).parent
WIFI_FLOORPLAN = HERE.parent / "wifi detection" / "host" / "floorplan.json"
SLOTS = {"corners": ["front-left", "front-right", "back-right", "back-left"],
         "edges": ["front", "right", "back", "left"]}       # same order round the zone: front -> front-left, ...


def _bounds(plan):
    xs = [p[0] for r in plan.get("rooms", []) for p in r["polygon"]]
    ys = [p[1] for r in plan.get("rooms", []) for p in r["polygon"]]
    if not xs:
        xs = [p[0] for p in plan["boards"].values()]
        ys = [p[1] for p in plan["boards"].values()]
    return min(xs), min(ys), max(xs), max(ys)


def _place(plan):
    """(layout, distance out from the tape, {mac: slot index}) of the plan's boards."""
    x0, y0, x1, y1 = _bounds(plan)
    cx, cy, hw, hd = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    rel = {m: ((x - cx) / hw, (y - cy) / hd) for m, (x, y) in plan["boards"].items()}
    corners = all(abs(u) > 0.5 and abs(v) > 0.5 for u, v in rel.values())
    slots, outs = {}, []
    for m, (u, v) in rel.items():
        x, y = plan["boards"][m]
        if corners:
            slots[m] = {(-1, -1): 0, (1, -1): 1, (1, 1): 2, (-1, 1): 3}[(1 if u > 0 else -1, 1 if v > 0 else -1)]
            outs.append(max(x0 - x, x - x1))
        elif abs(v) >= abs(u):
            slots[m] = 0 if v < 0 else 2
            outs.append(y0 - y if v < 0 else y - y1)
        else:
            slots[m] = 1 if u > 0 else 3
            outs.append(x - x1 if u > 0 else x0 - x)
    if len(set(slots.values())) != len(slots):
        raise ValueError("two boards are on the same corner or edge: place them in floorplan.json by hand")
    return ("corners" if corners else "edges"), sum(outs) / len(outs), slots


def floorplan_for_area(plan: dict, w: float, d: float, layout: str | None = None, out_m: float | None = None) -> dict:
    """A copy of `plan` for a w x d m area (origin front-left, y away from the front)."""
    old_layout, old_out, slots = _place(plan)
    layout = layout or old_layout
    if out_m is None:       # keep the boards as far from the tape; a corner's along its diagonal
        out_m = old_out if layout == old_layout else old_out / math.sqrt(2) if layout == "corners" else old_out * math.sqrt(2)
    o = out_m
    spots = {"corners": [(-o, -o), (w + o, -o), (w + o, d + o), (-o, d + o)],
             "edges": [(w / 2, -o), (w + o, d / 2), (w / 2, d + o), (-o, d / 2)]}[layout]
    out = copy.deepcopy(plan)
    out["boards"] = {m: [round(spots[i][0], 3), round(spots[i][1], 3)] for m, i in slots.items()}
    out["rooms"] = [{"name": "Search area", "polygon": [[0, 0], [w, 0], [w, d], [0, d]]}]
    for s in out.get("stops", []):
        for k in ("a", "b"):
            s[k] = [round(min(max(s[k][0], 0.0), w), 3), round(min(max(s[k][1], 0.0), d), 3)]
    out["comment"] = (f"Search area {w * 100:.1f} x {d * 100:.1f} cm (set by wifi_area.py). Origin = front-left tape "
                      f"corner (laptop side); x to the right, y away from the laptop. Boards at waist height on the "
                      f"{layout}, {o * 100:.0f} cm out from the tape" + (" in x and in y." if layout == "corners" else "."))
    return out


def floorplan_path(cfg: dict) -> Path:
    path = Path(cfg.get("wifi_floorplan", WIFI_FLOORPLAN))
    return path if path.is_absolute() else HERE / path


def update_floorplan(w: float, d: float, layout: str | None = None, out_m: float | None = None) -> list[str]:
    """Rewrite the WiFi floor plan for a w x d m area; returns one line per board saying where it goes."""
    path = floorplan_path(json.loads(CONFIG.read_text()))
    plan = floorplan_for_area(json.loads(path.read_text()), w, d, layout, out_m)
    path.write_text(json.dumps(plan, indent=2))
    new_layout, _, slots = _place(plan)
    names = SLOTS[new_layout]
    return [f"board {m[-5:]}: {names[slots[m]]:11s} x {x * 100:6.1f} cm, y {y * 100:6.1f} cm"
            for m, (x, y) in sorted(plan["boards"].items(), key=lambda kv: slots[kv[0]])]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("width_cm", type=float, help="left to right")
    ap.add_argument("depth_cm", type=float, help="front (your side) to back")
    ap.add_argument("--layout", choices=["corners", "edges"], help="default: keep the current one")
    ap.add_argument("--out", type=float, help="cm from the tape to each board (default: keep the current)")
    args = ap.parse_args()
    lines = update_floorplan(args.width_cm / 100, args.depth_cm / 100, args.layout,
                             None if args.out is None else args.out / 100)
    print(f"WiFi floor plan set to {args.width_cm:g} x {args.depth_cm:g} cm (cm from the front-left tape corner):")
    print("\n".join("  " + s for s in lines))
    print("The server picks this up by itself. Put the boards there at waist height, then press Recalibrate.")


if __name__ == "__main__":
    main()
