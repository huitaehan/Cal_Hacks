"""Tests for the RuView Observatory bridge.  Run: python test_ruview_bridge.py"""

import json
import struct

from ruview_bridge import build_frame, ws_accept_key, ws_text_frame


def _state(detected: bool, ratio: float, rate: float = 45.0) -> dict:
    node = {
        "port": "COM10", "csi_rate": rate, "rssi": -60.0,
        "row": [1.0] * 28 + [0.0] + [1.0] * 28,  # 57 subcarriers incl. DC null
        "detector": {"ratio": ratio, "subcarrier_cv": [0.03] * 56},
    }
    return {"nodes": [node, dict(node, port="COM11", csi_rate=0.0, row=None)],
            "summary": {"detected": detected, "confidence": 0.8}}


def test_person_only_when_detected():
    assert build_frame(_state(False, 0.4))["persons"] == []
    f = build_frame(_state(True, 2.0))
    assert f["estimated_persons"] == 1 and f["classification"]["presence"] is True
    assert f["persons"][0]["pose"] == "walking"
    assert build_frame(_state(True, 1.1))["persons"][0]["pose"] == "standing"


def test_shapes_match_observatory():
    f = build_frame(_state(True, 1.2))
    assert f["type"] == "sensing_update"
    assert len(f["signal_field"]["values"]) == 400
    assert len(f["nodes"]) == 1, "nodes without CSI must be omitted"
    assert len(f["nodes"][0]["amplitude"]) == 64
    assert len(f["_observatory"]["per_subcarrier_variance"]) == 64
    json.dumps(f)  # must be JSON-serialisable


def test_websocket_handshake_and_framing():
    key = "dGhlIHNhbXBsZSBub25jZQ=="  # RFC 6455 section 1.3 example
    assert ws_accept_key(key) == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="
    small = ws_text_frame("hi")
    assert small[:2] == bytes([0x81, 2]) and small[2:] == b"hi"
    mid = ws_text_frame("x" * 300)
    assert mid[:2] == bytes([0x81, 126]) and struct.unpack("!H", mid[2:4])[0] == 300
    big = ws_text_frame("y" * 70000)
    assert big[:2] == bytes([0x81, 127]) and struct.unpack("!Q", big[2:10])[0] == 70000


if __name__ == "__main__":
    test_person_only_when_detected()
    test_shapes_match_observatory()
    test_websocket_handshake_and_framing()
    print("ok")
