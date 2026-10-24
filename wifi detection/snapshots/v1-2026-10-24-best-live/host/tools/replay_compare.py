"""
Replay a recording through both per-link detectors and compare them.

    python tools/replay_compare.py recordings/<file>.log [--target-pps N|auto] [--as-amplitude]

Handles raw-I/Q `CSI,` lines (single-peer firmware: one link per file) and the
multi-peer `CSIA,` / `RLY,` amplitude lines (every link in the mesh). Prints, per
link and per 10 s, the median/max ratio of the denoised detector (10 s
calibration, as live) and of ESPectre (no calibration), plus the share of time
each one reports motion.
"""

from __future__ import annotations

import argparse
import collections
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from espectre_link import EspectreLinkDetector  # noqa: E402
from presence_server import LinkDetector, Settings  # noqa: E402

PPS_CHOICES = (100, 50, 25, 20, 10)


def read_packets(path: Path, as_amplitude: bool):
    """Yield (t, link_key, payload, device_ts_us) where payload is raw I/Q bytes or amplitudes."""
    for line in path.open():
        t_s, rest = line.split(",", 1)
        t = float(t_s)
        p = rest.rstrip().split(",")
        try:
            if rest.startswith("CSI,") and len(p) >= 8:
                raw = bytes.fromhex(p[7])
                if len(raw) % 2:
                    continue
                if as_amplitude:
                    iq = np.frombuffer(raw, dtype=np.int8).astype(np.float32).reshape(-1, 2)
                    yield t, "peer->self", np.rint(np.sqrt((iq ** 2).sum(axis=1))), int(p[2])
                else:
                    yield t, "peer->self", raw, int(p[2])
            elif rest.startswith("CSIA,") and len(p) == 8:
                yield t, f"{p[1][-5:]}->self", np.frombuffer(bytes.fromhex(p[7]), np.uint8).astype(np.float32), int(p[3])
            elif rest.startswith("RLY,") and len(p) == 7:
                yield t, f"{p[2][-5:]}->{p[1][-5:]}", np.frombuffer(bytes.fromhex(p[6]), np.uint8).astype(np.float32), None
        except ValueError:
            continue


def pick_pps(rate: float) -> int:
    for pps in PPS_CHOICES:
        if rate >= 0.8 * pps:
            return pps
    return PPS_CHOICES[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording", type=Path)
    ap.add_argument("--target-pps", default="auto")
    ap.add_argument("--as-amplitude", action="store_true", help="convert raw I/Q to rounded amplitudes first")
    args = ap.parse_args()

    packets = list(read_packets(args.recording, args.as_amplitude))
    if not packets:
        sys.exit("no CSI packets in recording")
    t0 = packets[0][0]
    by_link = collections.defaultdict(list)
    for pkt in packets:
        by_link[pkt[1]].append(pkt)

    for key, pkts in sorted(by_link.items()):
        dur = max(1e-6, pkts[-1][0] - pkts[0][0])
        rate = len(pkts) / dur
        pps = pick_pps(rate) if args.target_pps == "auto" else int(args.target_pps)
        ours = LinkDetector(Settings(10.0))
        esp = EspectreLinkDetector(target_pps=pps)
        rows = collections.defaultdict(lambda: {"ours": [], "esp": [], "ours_on": [], "esp_on": []})
        on = {"ours": 0, "esp": 0}
        n_eval = {"ours": 0, "esp": 0}
        for i, (t, _, payload, dev_ts) in enumerate(pkts):
            amp = payload if not isinstance(payload, bytes) else np.sqrt(
                (np.frombuffer(payload, np.int8).astype(np.float32).reshape(-1, 2) ** 2).sum(axis=1))
            ours.push(t, amp)
            esp.push(t, payload, device_ts_us=dev_ts)
            if i % 5:
                continue
            ours.update(t)
            esp.update(t)
            bucket = int((t - t0) // 10) * 10
            so, se = ours.snapshot(t), esp.snapshot(t)
            if so["ratio"] is not None:
                rows[bucket]["ours"].append(so["ratio"])
                rows[bucket]["ours_on"].append(bool(so["detected"]))
                n_eval["ours"] += 1
                on["ours"] += bool(so["detected"])
            if se["ratio"] is not None:
                rows[bucket]["esp"].append(se["ratio"])
                rows[bucket]["esp_on"].append(bool(se["detected"]))
                n_eval["esp"] += 1
                on["esp"] += bool(se["detected"])

        print(f"\n== link {key}: {len(pkts)} packets, {rate:.1f} pps, ESPectre target {pps} pps")
        print("  time    denoised med/max  on%     espectre med/max  on%")
        for b in sorted(rows):
            r = rows[b]
            fo = f"{np.median(r['ours']):5.2f}/{np.max(r['ours']):5.2f} {100 * np.mean(r['ours_on']):4.0f}" \
                if r["ours"] else "   -/-        -"
            fe = f"{np.median(r['esp']):5.2f}/{np.max(r['esp']):5.2f} {100 * np.mean(r['esp_on']):4.0f}" \
                if r["esp"] else "   -/-        -"
            print(f"  +{b:4d}s  {fo:>20}     {fe:>20}")
        for k in ("ours", "esp"):
            share = 100.0 * on[k] / n_eval[k] if n_eval[k] else float("nan")
            print(f"  {k:>4}: motion {share:5.1f}% of {n_eval[k]} evaluations")


if __name__ == "__main__":
    main()
