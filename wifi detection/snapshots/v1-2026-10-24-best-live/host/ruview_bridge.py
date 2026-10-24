"""
Feed RuView's 3D Observatory (RuView/ui/observatory.html) from the presence server.

The Observatory looks for a RuView sensing server on its own origin: it calls
GET /health, then reads `sensing_update` JSON frames from the /ws/sensing
WebSocket. This module builds those frames from App.state() and implements the
server side of that WebSocket, so the Observatory runs on the ESP32-C5 data.

What is measured vs. drawn:
  measured  - presence, motion level, RSSI, per-subcarrier amplitude/variation
  drawn     - the figure's body and limb animation (RuView's pose system picks a
              canned standing/walking animation from `pose` + `motion_score`)
  fixed     - the figure's position: one link cannot locate a person, so it is
              placed at the link midpoint until multi-node localization exists
"""

from __future__ import annotations

import base64
import hashlib
import struct
import time

import numpy as np

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"  # fixed by RFC 6455
GRID = 20              # Observatory signal_field is 20 x 20 cells of 0.5 m
CELL_M = 0.5
N_SUBCARRIERS = 64     # Observatory visualisations assume 64 subcarriers


# ---- WebSocket (RFC 6455, server -> client text frames only) ---------------

def ws_accept_key(client_key: str) -> str:
    return base64.b64encode(hashlib.sha1((client_key + WS_GUID).encode()).digest()).decode()


def ws_text_frame(text: str) -> bytes:
    payload = text.encode("utf-8")
    n = len(payload)
    if n < 126:
        header = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        header = struct.pack("!BBH", 0x81, 126, n)
    else:
        header = struct.pack("!BBQ", 0x81, 127, n)
    return header + payload


# ---- Frame building ---------------------------------------------------------

def _resample(values, n: int = N_SUBCARRIERS) -> list[float]:
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return [0.0] * n
    if v.size == n:
        return v.round(4).tolist()
    x_old = np.linspace(0.0, 1.0, v.size)
    return np.interp(np.linspace(0.0, 1.0, n), x_old, v).round(4).tolist()


def _link_field(a: tuple[float, float], b: tuple[float, float], intensity: float) -> list[float]:
    """Heat along the A-B link (its sensing zone), scaled by measured motion."""
    ax, az = a
    bx, bz = b
    abx, abz = bx - ax, bz - az
    seg2 = abx * abx + abz * abz or 1.0
    vals = []
    for iz in range(GRID):
        for ix in range(GRID):
            x = (ix - GRID / 2) * CELL_M
            z = (iz - GRID / 2) * CELL_M
            t = max(0.0, min(1.0, ((x - ax) * abx + (z - az) * abz) / seg2))
            dx, dz = x - (ax + t * abx), z - (az + t * abz)
            d2 = dx * dx + dz * dz
            vals.append(round(min(1.0, 0.04 + intensity * float(np.exp(-d2 / (2 * 0.6 ** 2)))), 4))
    return vals


def build_frame(state: dict, link_length_m: float = 3.0) -> dict:
    nodes = state["nodes"]
    summary = state["summary"]
    half = link_length_m / 2.0
    positions = [(-half, 0.0), (half, 0.0)]

    live = [n for n in nodes if n["csi_rate"] > 0]
    links = state.get("links") or []
    ratios = [l["ratio"] for l in links if l.get("ratio") is not None] or \
             [n["detector"]["ratio"] for n in live if n["detector"]["ratio"] is not None]
    ratio = max(ratios) if ratios else 0.0
    rssis = [n["rssi"] for n in live if n["rssi"] is not None]
    mean_rssi = float(np.mean(rssis)) if rssis else -90.0
    detected = bool(summary["detected"])

    frame_nodes = []
    for i, n in enumerate(nodes):
        if n["csi_rate"] <= 0 or not n["row"]:
            continue
        x, z = positions[i % 2]
        frame_nodes.append({
            "node_id": i + 1,
            "rssi_dbm": n["rssi"],
            "position": [x, 0.0, z],
            "amplitude": [round(0.45 * a, 4) for a in _resample(n["row"])],
            "subcarrier_count": N_SUBCARRIERS,
        })

    cv = next((n["detector"].get("subcarrier_cv") for n in live if n["detector"].get("subcarrier_cv")), None)
    variance = _resample(np.square(cv)) if cv else [0.0] * N_SUBCARRIERS

    motion_power = float(np.clip(0.02 + 0.15 * ratio, 0.0, 1.0))
    if detected:
        motion_level = "active" if ratio >= 1.0 else "present_still"
        confidence = float(summary["confidence"])
    else:
        motion_level = "absent"
        confidence = float(np.clip(1.0 - ratio / 2.0, 0.5, 0.99))

    loc = state.get("localization") or {}
    persons = []
    if detected:
        # Multi-link estimate when available (centred on the boards); else the link midpoint.
        px = pz = 0.0
        if loc.get("person") and loc.get("center"):
            px = loc["person"][0] - loc["center"][0]
            pz = loc["person"][1] - loc["center"][1]
        persons.append({
            "id": "p0",
            "position": [round(px, 3), 0.0, round(pz, 3)],
            "motion_score": float(np.clip(ratio * 60.0, 5.0, 200.0)),
            "pose": "walking" if ratio >= 1.5 else "standing",
            "facing": 0.0,
        })

    return {
        "type": "sensing_update",
        "timestamp": time.time(),
        "source": "esp32c5",
        "scenario": "live",
        "nodes": frame_nodes,
        "features": {
            "mean_rssi": mean_rssi,
            "variance": round(ratio, 4),
            "std": round(float(np.sqrt(ratio)), 4),
            "motion_band_power": round(motion_power, 4),
            "breathing_band_power": 0.0,
            "dominant_freq_hz": 0.0,
            "spectral_power": round(motion_power, 4),
        },
        "classification": {"motion_level": motion_level, "presence": detected, "confidence": round(confidence, 3)},
        "signal_field": {
            "grid_size": [GRID, 1, GRID],
            "values": ([round(0.04 + 0.9 * v, 4) for v in loc["obs_field"]] if loc.get("obs_field") else
                       _link_field(positions[0], positions[1], float(np.clip(0.12 + 0.25 * ratio, 0.0, 0.9)))),
        },
        # Not measured by this setup; zero confidence keeps the HUD from showing invented vitals.
        "vital_signs": {"breathing_rate_bpm": 0, "heart_rate_bpm": 0,
                        "breathing_confidence": 0, "heart_rate_confidence": 0},
        "persons": persons,
        "estimated_persons": len(persons),
        "edge_modules": {},
        "_observatory": {"subcarrier_iq": [], "per_subcarrier_variance": variance},
    }
