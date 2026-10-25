"""The WiFi floor plan follows a new area size: each board keeps its own corner (or edge middle) and its
distance out from the tape, the search area becomes the new rectangle, and walls and settings are kept."""
import math

from wifi_area import floorplan_for_area

CORNERS = {
    "comment": "old", "rooms": [{"name": "Search area", "polygon": [[0, 0], [1.51, 0], [1.51, 1.375], [0, 1.375]]}],
    "boards": {"fl": [-0.18, -0.18], "fr": [1.69, -0.18], "br": [1.69, 1.555], "bl": [-0.18, 1.555]},
    "walls": [], "motion_half_life_s": 8,
    "stops": [{"name": "Search area", "a": [1.255, 0.295], "b": [0.25, 0.295], "listen_s": 15}],
}
EDGES = dict(CORNERS, boards={"f": [0.755, -0.25], "r": [1.76, 0.6875], "b": [0.755, 1.625], "l": [-0.25, 0.6875]})


def test_corner_boards_follow_a_new_area_and_keep_their_corners():
    out = floorplan_for_area(CORNERS, 1.97, 2.38)
    assert out["boards"] == {"fl": [-0.18, -0.18], "fr": [2.15, -0.18], "br": [2.15, 2.56], "bl": [-0.18, 2.56]}
    assert out["rooms"][0]["polygon"] == [[0, 0], [1.97, 0], [1.97, 2.38], [0, 2.38]]
    assert out["walls"] == [] and out["motion_half_life_s"] == 8
    assert CORNERS["boards"]["br"] == [1.69, 1.555], "the input plan is left alone"


def test_boards_can_sit_right_on_the_tape_corners():
    assert floorplan_for_area(CORNERS, 1.97, 2.38, out_m=0.0)["boards"]["br"] == [1.97, 2.38]


def test_edge_middle_boards_follow_the_area_too():
    out = floorplan_for_area(EDGES, 1.97, 2.38)
    assert out["boards"] == {"f": [0.985, -0.25], "r": [2.22, 1.19], "b": [0.985, 2.63], "l": [-0.25, 1.19]}


def test_switching_layout_keeps_the_order_round_the_zone_and_the_distance_out():
    # front -> front-left, right -> front-right, back -> back-right, left -> back-left
    out = floorplan_for_area(EDGES, 1.97, 2.38, layout="corners")
    o = round(0.25 / math.sqrt(2), 3)                      # same 25 cm out, now along the corner's diagonal
    assert out["boards"] == {"f": [-o, -o], "r": [round(1.97 + o, 3), -o], "b": [round(1.97 + o, 3), round(2.38 + o, 3)],
                             "l": [-o, round(2.38 + o, 3)]}, out["boards"]


def test_stops_stay_inside_a_smaller_area():
    out = floorplan_for_area(CORNERS, 1.0, 0.25)
    for s in out["stops"]:
        for x, y in (s["a"], s["b"]):
            assert 0 <= x <= 1.0 and 0 <= y <= 0.25, s


if __name__ == "__main__":
    test_corner_boards_follow_a_new_area_and_keep_their_corners()
    test_boards_can_sit_right_on_the_tape_corners()
    test_edge_middle_boards_follow_the_area_too()
    test_switching_layout_keeps_the_order_round_the_zone_and_the_distance_out()
    test_stops_stay_inside_a_smaller_area()
    print("ok")
