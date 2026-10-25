"""
Floor plan of the search area: rooms, fixed boards, walls and the stop plan.

All coordinates are floor-plan metres (x, y) from the origin the robot team's
camera tracking uses. File shape (see floorplan.example.json):

    {"rooms":  [{"name": "Room A", "polygon": [[x, y], ...]}],
     "boards": {"<mac>": [x, y]},                  fixed anchor / gateway / parked robots
     "walls":  [[[x1, y1], [x2, y2]], ...],        drawing only
     "stops":  [{"name": "Room A", "a": [x, y], "b": [x, y], "listen_s": 15}],
     "motion_half_life_s": 8}                      optional: short motion memory for sentry mode
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass, field
from pathlib import Path


def _point(v, what: str) -> tuple[float, float]:
    try:
        x, y = v
        return float(x), float(y)
    except (TypeError, ValueError):
        raise ValueError(f"{what}: expected [x, y], got {v!r}")


@dataclass
class Room:
    name: str
    polygon: list[tuple[float, float]]

    def contains(self, x: float, y: float) -> bool:
        inside = False
        pts = self.polygon
        j = len(pts) - 1
        for i in range(len(pts)):
            xi, yi = pts[i]
            xj, yj = pts[j]
            if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                inside = not inside
            j = i
        return inside


@dataclass
class Stop:
    name: str
    a: tuple[float, float]
    b: tuple[float, float]
    listen_s: float = 15.0


@dataclass
class FloorPlan:
    rooms: list[Room]
    boards: dict[str, tuple[float, float]] = field(default_factory=dict)
    walls: list[tuple[tuple[float, float], tuple[float, float]]] = field(default_factory=list)
    stops: list[Stop] = field(default_factory=list)
    motion_half_life_s: float | None = None   # None: rooms use their default memory

    @classmethod
    def from_dict(cls, d) -> FloorPlan:
        try:
            return cls._from_dict(d)
        except (TypeError, AttributeError, KeyError, IndexError) as e:
            # Structural mistakes (a list where an object belongs, null numbers, ...)
            raise ValueError(f"unexpected structure: {e!r}") from e

    @classmethod
    def _from_dict(cls, d) -> FloorPlan:
        if not isinstance(d, dict):
            raise ValueError("floor plan must be a JSON object")
        rooms_raw = d.get("rooms")
        if not isinstance(rooms_raw, list) or not rooms_raw:
            raise ValueError("rooms: expected a non-empty list of {name, polygon}")
        rooms = []
        for i, r in enumerate(rooms_raw):
            if not isinstance(r, dict) or "polygon" not in r:
                raise ValueError(f"rooms[{i}]: expected {{name, polygon}}")
            poly = [_point(p, f"rooms[{i}].polygon") for p in r["polygon"]]
            if len(poly) < 3:
                raise ValueError(f"rooms[{i}].polygon: needs at least 3 points")
            rooms.append(Room(str(r.get("name", f"Room {i + 1}")), poly))
        boards = {str(k).lower(): _point(v, f"boards[{k}]") for k, v in (d.get("boards") or {}).items()}
        walls = [(_point(w[0], "walls"), _point(w[1], "walls")) for w in (d.get("walls") or [])]
        stops = []
        for i, s in enumerate(d.get("stops") or []):
            stops.append(Stop(str(s.get("name", f"Stop {i + 1}")), _point(s.get("a"), f"stops[{i}].a"),
                              _point(s.get("b"), f"stops[{i}].b"), float(s.get("listen_s", 15.0))))
        half_life = d.get("motion_half_life_s")
        if half_life is not None:
            half_life = float(half_life)
            if not half_life > 0:
                raise ValueError(f"motion_half_life_s: must be positive seconds, got {half_life!r}")
        return cls(rooms, boards, walls, stops, half_life)

    @classmethod
    def load(cls, path: Path) -> FloorPlan:
        try:
            data = json.loads(Path(path).read_text())
        except json.JSONDecodeError as e:
            raise ValueError(f"not valid JSON: {e}")
        return cls.from_dict(data)

    def to_dict(self) -> dict:
        d = {
            "rooms": [{"name": r.name, "polygon": [list(p) for p in r.polygon]} for r in self.rooms],
            "boards": {k: list(v) for k, v in self.boards.items()},
            "walls": [[list(a), list(b)] for a, b in self.walls],
            "stops": [{"name": s.name, "a": list(s.a), "b": list(s.b), "listen_s": s.listen_s} for s in self.stops],
        }
        if self.motion_half_life_s is not None:
            d["motion_half_life_s"] = self.motion_half_life_s
        return d


class FloorPlanWatcher:
    """Reloads the floor plan when the file changes; an invalid edit keeps the last valid plan."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._mtime = None
        self._plan: FloorPlan | None = None
        self._error: str | None = None
        # Several server threads read the plan; a reader must never see the new
        # mtime before the matching plan has finished loading.
        self._lock = threading.Lock()

    def current(self) -> tuple[FloorPlan | None, str | None]:
        with self._lock:
            try:
                mtime = self.path.stat().st_mtime
            except FileNotFoundError:
                return self._plan, self._error or f"no floor plan at {self.path.name}"
            if mtime != self._mtime:
                self._mtime = mtime
                try:
                    self._plan = FloorPlan.load(self.path)
                    self._error = None
                except Exception as e:  # never let an edit break a caller thread; report it instead
                    self._error = f"{self.path.name}: {e}"
            return self._plan, self._error
