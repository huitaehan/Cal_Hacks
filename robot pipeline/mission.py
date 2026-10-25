"""Mission logic: robots watch from their sentry spots; a WiFi detection sends the nearest one over.

    sentry   -> robots hold their spots; once all the robots the camera sees are home and settled, a WiFi
                position that persists dispatches
    dispatch -> the nearest robot drives to a point just short of the WiFi position (the others hold);
                if it stops making progress (blocked), it listens from where it is
    verify   -> it holds still and listens; detections nearby confirm it
    found    -> it stays with the person for a while, then everyone returns
    return   -> robots back to their spots; one that can't get home in time is parked where it is
    paused / manual (operator). While paused, only resume does anything.

`tick()` returns each robot's goal (x, y), or None for "stop and hold". Pure logic, no I/O.
"""
import math
from dataclasses import dataclass


@dataclass
class MissionConfig:
    sentry: dict                # robot -> (x, y)
    area: tuple                 # ((x0, y0), (x1, y1)); targets are clamped inside
    home_tol_m: float = 0.15    # a robot this close to its sentry spot is home
    arrive_tol_m: float = 0.10  # a dispatched robot this close to its target has arrived (driver stops at 0.08)
    settle_s: float = 5.0       # after the robots are home, WiFi links need a few still seconds
    detect_hold_s: float = 2.0  # a WiFi position must persist this long to dispatch
    standoff_m: float = 0.45    # stop this far short of the WiFi position
    retarget_m: float = 0.8     # re-aim if the WiFi position moves this far while driving
    stall_s: float = 4.0        # no 5 cm of progress for this long while dispatched: listen from there
    verify_s: float = 8.0       # hold still next to the spot this long
    verify_settle_s: float = 3.5   # the WiFi server ignores a robot's links until it's been still 3 s
    verify_radius_m: float = 1.5   # detections this close to the spot count as confirmation
    verify_min_frac: float = 0.3   # fraction of the listening time with a detection nearby
    found_hold_s: float = 20.0  # stay with the person this long (0: until recalled)
    found_radius_m: float = 1.0
    found_ignore_s: float = 60.0   # don't re-dispatch to a found spot for this long
    return_timeout_s: float = 20.0  # a robot not home by then is parked where it is
    follow: bool = False        # follow mode: the nearest robot goes straight to each WiFi position and keeps
                                # re-aiming; no settling, holding, listening or ignored spots
    follow_retarget_m: float = 0.25    # follow mode: re-aim once the position moves this far
    follow_lost_s: float = 8.0  # follow mode: no detection for this long, back to the spot


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Mission:
    def __init__(self, cfg: MissionConfig, snap=None):
        """snap: optional (x, y) -> (x, y) moving a target to reachable floor (planner.Planner.nearest_free)."""
        self.cfg = cfg
        self.snap = snap
        self.sentry = {r: tuple(snap(tuple(p)) or p) if snap else tuple(p) for r, p in cfg.sentry.items()}
        self.state = "sentry"
        self.robot = None           # the dispatched (or manually driven) robot
        self.target = None
        self.events: list = []      # (t, text)
        self.n_logged = 0           # events logged so far (the list keeps the last 50)
        self.found: list = []       # (t, x, y)
        self.parked: set = set()    # robots that couldn't get home: they hold where they are
        self._since = 0.0           # when the current state started
        self._home_since = None
        self._seen_since = None
        self._aimed_at = None       # WiFi position the current target was computed from
        self._best_d = math.inf     # dispatch progress
        self._progress_t = 0.0
        self._hits: list = []
        self._ticks = 0
        self._last_seen = 0.0       # follow mode: last WiFi position
        self._episode_found = False

    # operator commands
    def pause(self, now):
        self._set(now, "paused")
        self.note(now, "Paused: all robots stopped (p to resume)")

    def resume(self, now):
        self.parked.clear()
        self._set(now, "return")
        self.note(now, "Resumed: robots going to their spots")

    def recall(self, now):
        if self.state == "paused":
            self.note(now, "Ignored recall: paused (p to resume)")
            return
        self.parked.clear()
        self._set(now, "return")
        self.note(now, "Recalled: robots going to their spots")

    def goto(self, now, robot, xy):
        if self.state == "paused":
            self.note(now, "Ignored move: paused (p to resume)")
            return
        self.robot, self.target = robot, self._clamp(xy)
        self._set(now, "manual")
        self.note(now, f"Operator: robot {robot} to ({self.target[0]:.1f}, {self.target[1]:.1f})")

    def clear_found(self, now):
        self.found = []
        self.note(now, "Found list cleared")

    def note(self, now, text):
        self.events.append((now, text))
        del self.events[:-50]
        self.n_logged += 1

    # main step
    def tick(self, now, poses: dict, person):
        c = self.cfg
        pos = {r: (p.x, p.y) for r, p in poses.items()}
        home = lambda r: r in pos and _dist(pos[r], self.sentry[r]) <= c.home_tol_m
        holds = {r: (None if r in self.parked else self.sentry[r]) for r in self.sentry}

        if self.state == "paused":
            return {r: None for r in self.sentry}
        if c.follow:
            return self._follow(now, pos, person, home, holds)

        if self.state == "return":
            seen = [r for r in self.sentry if r in pos]
            if seen and all(home(r) for r in seen):
                self._set(now, "sentry")
                self.note(now, "Robots on their spots: watching")
            elif now - self._since >= c.return_timeout_s:
                late = [r for r in seen if not home(r)]
                self.parked.update(late)
                self._set(now, "sentry")
                self.note(now, f"Robot {', '.join(late) or '?'} not back after {c.return_timeout_s:.0f} s: "
                               f"holding where it is; watching")
                holds = {r: (None if r in self.parked else self.sentry[r]) for r in self.sentry}
            else:
                return dict(self.sentry)

        if self.state == "sentry":
            self.parked -= {r for r in self.parked if home(r)}
            seen = [r for r in self.sentry if r in pos]
            ready = bool(seen) and all(home(r) or r in self.parked for r in seen)
            if not ready:
                self._home_since = None
            elif self._home_since is None:
                self._home_since = now
            watching = self._home_since is not None and now - self._home_since >= c.settle_s
            p = person if watching and person is not None and not self._already_found(now, person) else None
            if p is None:
                self._seen_since = None
            else:
                self._seen_since = self._seen_since if self._seen_since is not None else now
                if now - self._seen_since >= c.detect_hold_s:
                    self.robot = min(seen, key=lambda r: _dist(pos[r], p))
                    self._aim(now, p, pos[self.robot])
                    self._set(now, "dispatch")
                    self.note(now, f"Motion near ({p[0]:.1f}, {p[1]:.1f}): sending robot {self.robot}")
            if self.state == "sentry":
                return holds

        goals = dict(holds)
        if self.state == "dispatch":
            if person is not None and _dist(person, self._aimed_at) > c.retarget_m and self.robot in pos:
                self._aim(now, person, pos[self.robot])
                self.note(now, f"Position moved: robot {self.robot} re-aimed to ({person[0]:.1f}, {person[1]:.1f})")
            d = _dist(pos[self.robot], self.target) if self.robot in pos else None
            if d is not None and d < self._best_d - 0.05:
                self._best_d, self._progress_t = d, now
            if d is not None and d <= c.arrive_tol_m:
                self._start_verify(now)
                self.note(now, f"Robot {self.robot} arrived: listening")
            elif now - self._progress_t >= c.stall_s:
                if self.robot in pos:
                    self.target = pos[self.robot]
                self._start_verify(now)
                self.note(now, f"Robot {self.robot} couldn't get closer (blocked or not seen): listening from here")
        elif self.state == "verify":
            if now - self._since >= c.verify_settle_s:
                self._ticks += 1
                if person is not None and _dist(person, self.target) <= c.verify_radius_m:
                    self._hits.append(person)
            if now - self._since >= c.verify_s:
                if self._ticks and len(self._hits) / self._ticks >= c.verify_min_frac:
                    x = sum(h[0] for h in self._hits) / len(self._hits)
                    y = sum(h[1] for h in self._hits) / len(self._hits)
                    self.found.append((now, x, y))
                    self._set(now, "found")
                    self.note(now, f"FOUND: person near ({x:.1f}, {y:.1f}); robot {self.robot} is staying with them")
                else:
                    a = self._aimed_at
                    self.note(now, f"Motion near ({a[0]:.1f}, {a[1]:.1f}) not confirmed; robots returning")
                    self._set(now, "return")
                    return dict(self.sentry)
        elif self.state == "found":
            if c.found_hold_s > 0 and now - self._since >= c.found_hold_s:
                self._set(now, "return")
                self.note(now, f"Robot {self.robot} returning to its spot")
                return dict(self.sentry)
        goals[self.robot] = self.target
        return goals

    def _follow(self, now, pos, person, home, holds):
        """Follow mode: follow -> found (once per episode) while the WiFi position lasts; return when it's lost.
        A detection also ends a manual move."""
        c = self.cfg
        seen = [r for r in self.sentry if r in pos]
        if person is not None and seen:
            self._last_seen = now
            if self.state not in ("follow", "found"):
                self.robot = min(seen, key=lambda r: _dist(pos[r], person))
                self._aim(now, person, pos[self.robot])
                self._episode_found = False
                self._set(now, "follow")
                self.note(now, f"Motion near ({person[0]:.1f}, {person[1]:.1f}): robot {self.robot} following")
            elif self.robot in pos and _dist(person, self._aimed_at) > c.follow_retarget_m:
                self._aim(now, person, pos[self.robot])
                self._set(now, "follow")
            if (self.state == "follow" and self.robot in pos
                    and _dist(pos[self.robot], self.target) <= c.arrive_tol_m):
                if not self._episode_found:
                    self._episode_found = True
                    self.found.append((now, person[0], person[1]))
                    self.note(now, f"FOUND: person near ({person[0]:.1f}, {person[1]:.1f}); "
                                   f"robot {self.robot} is with them")
                self._set(now, "found")
        elif self.state in ("follow", "found") and now - self._last_seen >= c.follow_lost_s:
            self._set(now, "return")
            self.note(now, f"No motion for {c.follow_lost_s:.0f} s: robot {self.robot} going back to its spot")

        if self.state in ("follow", "found", "manual"):
            goals = dict(holds)
            goals[self.robot] = self.target
            return goals
        if self.state == "return":
            back = [r for r in self.sentry if r in pos]
            if back and all(home(r) for r in back):
                self._set(now, "sentry")
                self.note(now, "Robots on their spots: watching")
            elif now - self._since >= c.return_timeout_s:
                self.parked.update(r for r in back if not home(r))
                self._set(now, "sentry")
            else:
                return dict(self.sentry)
        self.parked -= {r for r in self.parked if home(r)}
        return {r: (None if r in self.parked else self.sentry[r]) for r in self.sentry}

    def _set(self, now, state):
        if self.state == "sentry" and state != "sentry":
            self._home_since = self._seen_since = None    # settle and re-detect after every excursion
        self.state, self._since = state, now

    def _start_verify(self, now):
        self._set(now, "verify")
        self._hits, self._ticks = [], 0

    def _aim(self, now, person, robot_xy):
        vx, vy = robot_xy[0] - person[0], robot_xy[1] - person[1]
        d = math.hypot(vx, vy)
        s = self.cfg.standoff_m
        t = robot_xy if d <= s else (person[0] + vx / d * s, person[1] + vy / d * s)
        self.target = self._clamp(t)
        self._aimed_at = tuple(person)
        self._best_d, self._progress_t = math.inf, now

    def _clamp(self, xy):
        (x0, y0), (x1, y1) = self.cfg.area
        xy = (min(max(xy[0], x0), x1), min(max(xy[1], y0), y1))
        if self.snap is not None:
            xy = tuple(self.snap(xy) or xy)
        return xy

    def _already_found(self, now, p):
        return any(now - t < self.cfg.found_ignore_s and _dist((x, y), p) <= self.cfg.found_radius_m
                   for t, x, y in self.found)
