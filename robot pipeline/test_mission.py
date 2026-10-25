"""Mission logic: sentry -> dispatch the nearest robot to a WiFi detection -> verify -> found -> return."""
import math

from mission import Mission, MissionConfig
from tracker import Pose

SENTRY = {"A": (4.5, 2.0), "B": (0.0, 2.0)}


def cfg(**kw):
    return MissionConfig(sentry=SENTRY, area=((0.0, 0.0), (4.5, 4.0)), **kw)


class World:
    """Robots teleport to their goals after `travel_s` (enough for mission logic)."""

    def __init__(self, mission, travel_s=2.0):
        self.m, self.travel_s, self.t = mission, travel_s, 0.0
        self.pos = dict(SENTRY)
        self.goal_since = {}
        self.goals = {}
        self.stuck = set()          # robots that can't move (blocked)

    def run(self, seconds, person=None, dt=0.25):
        for _ in range(int(seconds / dt)):
            self.t += dt
            poses = {r: Pose(x, y, 90.0, self.t) for r, (x, y) in self.pos.items()}
            p = person(self.t) if callable(person) else person
            self.goals = self.m.tick(self.t, poses, p)
            for r, g in self.goals.items():
                if g is None or r in self.stuck or r not in self.pos:
                    continue
                if self.goal_since.get(r, (None,))[0] != g:
                    self.goal_since[r] = (g, self.t)
                if self.t - self.goal_since[r][1] >= self.travel_s:
                    self.pos[r] = g
        return self.goals


def test_sentry_holds_spots_and_ignores_detections_until_settled():
    m = Mission(cfg(settle_s=5.0))
    w = World(m)
    goals = w.run(3.0, person=(3.0, 2.0))
    assert m.state == "sentry" and goals == SENTRY, (m.state, goals)


def test_a_short_blip_does_not_dispatch():
    m = Mission(cfg(settle_s=1.0, detect_hold_s=2.0))
    w = World(m)
    w.run(2.0)
    w.run(1.0, person=(3.0, 2.0))
    w.run(2.0)
    assert m.state == "sentry"


def test_persistent_detection_sends_the_nearest_robot_short_of_the_person():
    m = Mission(cfg(settle_s=1.0, detect_hold_s=2.0, standoff_m=0.45))
    w = World(m, travel_s=100)         # don't arrive yet
    w.run(2.0)
    goals = w.run(2.5, person=(3.5, 2.0))
    assert m.state == "dispatch" and m.robot == "A", (m.state, m.robot)
    gx, gy = goals["A"]
    assert abs(math.hypot(gx - 3.5, gy - 2.0) - 0.45) < 1e-6 and gx > 3.5, goals   # on A's side
    assert goals["B"] == SENTRY["B"]
    assert any("sending robot A" in e for _, e in m.events), m.events


def test_confirmed_detection_is_found_then_robot_returns_and_spot_is_ignored():
    m = Mission(cfg(settle_s=1.0, detect_hold_s=1.0, verify_s=6.0, verify_settle_s=2.0, found_hold_s=5.0,
                    found_ignore_s=60.0))
    w = World(m, travel_s=1.0)
    w.run(2.0)
    w.run(3.0, person=(1.0, 2.5))            # B is nearer: dispatch, arrive, start verifying
    assert m.robot == "B" and m.state == "verify", m.state
    w.run(6.0, person=(1.1, 2.4))            # still detected while B listens next to them
    assert m.state == "found" and len(m.found) == 1, m.state
    w.run(6.0, person=(1.1, 2.4))            # stays with them for found_hold_s, then returns
    w.run(4.0, person=(1.1, 2.4))
    assert m.state == "sentry" and w.pos["B"] == SENTRY["B"], (m.state, w.pos)
    w.run(5.0, person=(1.1, 2.4))            # the same person again: already found, no new dispatch
    assert m.state == "sentry"
    w.run(4.0, person=(3.6, 3.0))            # someone elsewhere: dispatch
    assert m.state in ("dispatch", "verify") and m.robot == "A"


def test_unconfirmed_detection_returns_without_marking_found():
    m = Mission(cfg(settle_s=1.0, detect_hold_s=1.0, verify_s=5.0, verify_settle_s=2.0))
    w = World(m, travel_s=1.0)
    w.run(2.0)
    w.run(3.0, person=(3.0, 1.0))
    assert m.state == "verify"
    w.run(6.0)                               # nothing while A listens
    assert m.state in ("return", "sentry") and m.found == [], (m.state, m.found)
    assert any("not confirmed" in e for _, e in m.events)


def test_pause_stops_everything_and_resume_goes_back_to_sentry():
    m = Mission(cfg())
    w = World(m)
    m.pause(0.0)
    goals = w.run(1.0, person=(3.0, 2.0))
    assert m.state == "paused" and all(g is None for g in goals.values())
    m.resume(w.t)
    goals = w.run(0.5)
    assert m.state == "sentry" and goals == SENTRY


def test_manual_goto_sends_one_robot_and_recall_brings_it_back():
    m = Mission(cfg())
    w = World(m, travel_s=0.5)
    m.goto(0.0, "A", (2.0, 3.0))
    goals = w.run(1.0)
    assert m.state == "manual" and goals["A"] == (2.0, 3.0) and goals["B"] == SENTRY["B"]
    m.recall(w.t)
    goals = w.run(1.0)
    assert goals["A"] == SENTRY["A"] and m.state in ("return", "sentry")


def test_targets_stay_inside_the_area():
    m = Mission(cfg(settle_s=0.5, detect_hold_s=0.5, standoff_m=0.45))
    w = World(m, travel_s=100)
    w.run(1.0)
    goals = w.run(1.0, person=(4.4, 3.9))
    gx, gy = goals[m.robot]
    assert 0.0 <= gx <= 4.5 and 0.0 <= gy <= 4.0, goals


def test_clicks_and_recall_are_ignored_while_paused():
    """Review finding: the click that focuses the window used to un-pause and drive a robot."""
    m = Mission(cfg())
    w = World(m)
    m.pause(0.0)
    m.goto(0.1, "A", (2.0, 2.0))
    m.recall(0.2)
    goals = w.run(1.0)
    assert m.state == "paused" and all(g is None for g in goals.values()), (m.state, goals)


def test_after_returning_robots_settle_again_before_the_next_dispatch():
    """Review finding: the settle wait was skipped after a return, so the robot was sent straight back out."""
    m = Mission(cfg(settle_s=5.0, detect_hold_s=2.0, verify_s=3.0, verify_settle_s=1.0))
    w = World(m, travel_s=1.0)
    w.run(6.0)
    w.run(3.0, person=(3.0, 1.0))
    assert m.state == "dispatch"
    w.run(5.0)                                   # nothing confirmed: return, home at ~13.75 s
    assert m.state == "sentry", m.state
    w.run(4.0, person=(3.0, 1.0))                # within the new 5 s settle: no dispatch yet
    assert m.state == "sentry", m.state


def test_a_blocked_robot_gives_up_and_listens_where_it_is():
    m = Mission(cfg(settle_s=1.0, detect_hold_s=1.0, stall_s=4.0, verify_s=3.0, verify_settle_s=1.0))
    w = World(m, travel_s=1.0)
    w.stuck = {"A"}
    w.run(2.0)
    w.run(2.0, person=(3.0, 1.0))
    assert m.state == "dispatch" and m.robot == "A"
    w.run(5.0)
    assert m.state == "verify" and m.target == SENTRY["A"], (m.state, m.target)
    assert any("couldn't get closer" in e for _, e in m.events), m.events


def test_return_parks_a_robot_that_cant_get_home_and_watches_anyway():
    m = Mission(cfg(return_timeout_s=10.0, settle_s=1.0, detect_hold_s=1.0))
    w = World(m, travel_s=0.5)
    m.goto(0.0, "B", (2.0, 3.0))
    w.run(1.0)
    w.stuck = {"B"}
    m.recall(w.t)
    w.run(5.0)
    assert m.state == "return"
    goals = w.run(6.0)
    assert m.state == "sentry" and goals["B"] is None, (m.state, goals)    # B holds where it is
    assert any("not back" in e for _, e in m.events), m.events
    w.run(3.0, person=(3.6, 3.0))
    assert m.state in ("dispatch", "verify") and m.robot == "A", m.state


def test_a_robot_the_camera_cant_see_doesnt_block_watching():
    m = Mission(cfg(settle_s=1.0, detect_hold_s=1.0))
    w = World(m)
    del w.pos["B"]
    w.run(4.0, person=(3.6, 2.5))
    assert m.state in ("dispatch", "verify") and m.robot == "A", m.state


def test_targets_are_moved_off_walls():
    """A WiFi position on (or behind) a cardboard wall: aim at the nearest free floor instead."""
    snap = lambda xy: (min(xy[0], 3.0), xy[1])          # pretend everything past x = 3 is wall
    m = Mission(cfg(settle_s=0.5, detect_hold_s=0.5, standoff_m=0.0), snap=snap)
    w = World(m, travel_s=100)
    w.pos["A"] = m.sentry["A"]                           # its spot moved off the wall too
    w.run(1.0)
    goals = w.run(1.0, person=(3.6, 2.0))
    assert goals[m.robot] == (3.0, 2.0), goals
    m.goto(w.t, "B", (3.9, 1.0))
    assert m.target == (3.0, 1.0), m.target


def test_sentry_spots_are_moved_off_walls():
    """A sentry spot inside a wall's clearance: the robot can only park beside it, so that's its spot."""
    m = Mission(cfg(), snap=lambda xy: (min(xy[0], 3.0), xy[1]))
    assert m.sentry == {"A": (3.0, 2.0), "B": (0.0, 2.0)}, m.sentry


def test_follow_mode_goes_straight_to_the_person_and_tracks_them():
    m = Mission(cfg(follow=True))
    w = World(m)
    goals = w.run(0.25, person=(3.0, 2.0))          # first tick: no settling, no hold
    assert m.state == "follow" and m.robot == "A", m.state
    assert math.dist(goals["A"], (3.45, 2.0)) < 1e-6, goals      # 45 cm short, on A's side
    assert goals["B"] == SENTRY["B"]
    goals = w.run(0.25, person=(3.0, 3.0))          # the person moved 1 m: re-aimed at once
    assert abs(math.dist(goals["A"], (3.0, 3.0)) - 0.45) < 1e-6, goals


def test_follow_mode_marks_found_once_and_goes_home_after_losing_the_person():
    m = Mission(cfg(follow=True, follow_lost_s=3.0))
    w = World(m, travel_s=1.0)
    w.run(2.0, person=(3.0, 2.0))                   # the robot gets there after 1 s
    assert m.state == "found" and len(m.found) == 1, (m.state, m.found)
    w.run(2.0, person=(3.05, 2.0))                  # small moves: stays, no second FOUND
    assert len(m.found) == 1
    goals = w.run(2.0)                              # lost for 2 s: waits next to the spot
    assert m.state == "found" and goals["A"] == m.target, (m.state, goals)
    goals = w.run(1.5)                              # lost for 3.5 s: back to its spot
    assert m.state in ("return", "sentry") and goals["A"] == SENTRY["A"], (m.state, goals)


def test_follow_mode_takes_over_from_a_manual_move():
    m = Mission(cfg(follow=True))
    w = World(m)
    m.goto(0.0, "A", (1.0, 1.0))
    w.run(1.0)
    assert m.state == "manual"
    w.run(0.25, person=(3.0, 2.0))
    assert m.state == "follow"


def test_follow_mode_pause_still_stops_everything():
    m = Mission(cfg(follow=True))
    w = World(m)
    m.pause(0.0)
    goals = w.run(0.5, person=(3.0, 2.0))
    assert m.state == "paused" and all(g is None for g in goals.values()), goals


if __name__ == "__main__":
    test_sentry_spots_are_moved_off_walls()
    test_targets_are_moved_off_walls()
    test_clicks_and_recall_are_ignored_while_paused()
    test_after_returning_robots_settle_again_before_the_next_dispatch()
    test_a_blocked_robot_gives_up_and_listens_where_it_is()
    test_return_parks_a_robot_that_cant_get_home_and_watches_anyway()
    test_a_robot_the_camera_cant_see_doesnt_block_watching()
    test_sentry_holds_spots_and_ignores_detections_until_settled()
    test_a_short_blip_does_not_dispatch()
    test_persistent_detection_sends_the_nearest_robot_short_of_the_person()
    test_confirmed_detection_is_found_then_robot_returns_and_spot_is_ignored()
    test_unconfirmed_detection_returns_without_marking_found()
    test_pause_stops_everything_and_resume_goes_back_to_sentry()
    test_manual_goto_sends_one_robot_and_recall_brings_it_back()
    test_targets_stay_inside_the_area()
    test_follow_mode_goes_straight_to_the_person_and_tracks_them()
    test_follow_mode_marks_found_once_and_goes_home_after_losing_the_person()
    test_follow_mode_takes_over_from_a_manual_move()
    test_follow_mode_pause_still_stops_everything()
    print("ok")
