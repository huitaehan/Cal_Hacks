"""
Kalman filter for the person marker.

The localizer's raw x, y estimate jumps around (a median 0.17 m between 0.2 s
updates in the live test while one person walked inside). A constant-velocity
Kalman filter smooths it while tracking walking speed, so it trails a walker
less than plain averaging would. A detection more than a couple of seconds
after the last one starts a fresh track instead of gliding across the area.

Simulated (0.17 m noise, 0.2 s steps): versus the old 0.6/0.4 average it jitters
24 % less while standing (0.068 vs 0.089 m), keeps up with a 0.5 m/s walk instead
of trailing 0.1 m, and has a similar error through turns (0.22 vs 0.20 m rms),
though it overshoots more on an instant reversal (0.66 vs 0.45 m worst). A 1 s
average was calmer still (0.037 m) but trailed a walker by 0.42 m.
"""

from __future__ import annotations

import numpy as np


class PositionKalman:
    def __init__(self, meas_std_m: float = 0.25, accel_std: float = 0.5, reset_after_s: float = 2.0):
        self.r2 = meas_std_m ** 2
        self.q2 = accel_std ** 2
        self.reset_after_s = reset_after_s
        self.s = None        # state [x, y, vx, vy]
        self.P = None
        self.t = None

    def update(self, z, t: float) -> tuple[float, float]:
        z = np.asarray(z, dtype=float)
        if self.s is None or t - self.t > self.reset_after_s:
            self.s = np.array([z[0], z[1], 0.0, 0.0])
            self.P = np.diag([self.r2, self.r2, 1.0, 1.0])
            self.t = t
            return float(z[0]), float(z[1])
        dt = max(t - self.t, 1e-3)
        self.t = t
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        Q = np.zeros((4, 4))
        Q[0, 0] = Q[1, 1] = dt ** 4 / 4
        Q[0, 2] = Q[2, 0] = Q[1, 3] = Q[3, 1] = dt ** 3 / 2
        Q[2, 2] = Q[3, 3] = dt ** 2
        self.s = F @ self.s
        self.P = F @ self.P @ F.T + Q * self.q2
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        S = H @ self.P @ H.T + np.eye(2) * self.r2
        K = self.P @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ (z - H @ self.s)
        self.P = (np.eye(4) - K @ H) @ self.P
        return float(self.s[0]), float(self.s[1])
