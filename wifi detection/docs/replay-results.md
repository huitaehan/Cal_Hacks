# Detector replay gate: ESPectre vs denoised (2026-10-24)

Recordings: `host/recordings/20261024_010613_COM13.log` and `20261024_010614_COM15.log`. These are two
single-peer links at -60 to -73 dBm and 16–48 packets/s. A person walked through at about 70–110 s and
again at 120–145 s; the room was quiet at 0–60 s (other people further away).
Tool: `host/tools/replay_compare.py`. "on%" = share of evaluations reporting motion after each detector's
debounce. The denoised detector calibrates for 10 s first; ESPectre never calibrates.

| Link | Detector | Walk 1 on% | Walk 2 on% | Quiet 0–60 s on% |
|---|---|---|---|---|
| COM13 | denoised | 32–100 | 63–85 | 0 |
| COM13 | ESPectre @20 slots/s | 0–56 (caught) | 23–49 (caught) | **0** |
| COM15 | denoised | 98–100 | 33–91 | **31–37** in two 10 s blocks |
| COM15 | ESPectre @20 slots/s | 24–72 (caught) | 0–23 (caught) | **0** |

- Amplitude-only input (the multi-peer firmware format) scores the same as raw I/Q, within ±0.4 on a
  few medians and with identical on%.
- At 25 slots/s these links (16–25 pps) fall below ESPectre's 70% slot-coverage rule. Readiness flickers
  and its 4-in-a-row debounce rarely completes (on% 1–3%). **Matching the slot rate to the link's real
  packet rate is essential.**

## Decision

**ESPectre wins** (catches both walks on both links, no false detection in the quiet stretch, no
calibration). Live settings:

- Per link, ESPectre's slot rate is chosen from the link's measured rate: the largest of 100/50/25/20/10
  that is ≤ 0.9 × the packet rate.
- ESPectre's firmware debounce is applied: 4 motion evaluations to switch on, 3 to switch off, every
  250 ms.
- The denoised detector stays available (`--detector denoised`). Survey mode (plan Task 7) is not needed.
- 100 Hz firmware (plan Task 8) is not needed: per-link slot rates make 8–50 pps links usable.

ESPectre is conservative while someone moves (on 20–70% of the time). Room evidence therefore builds over
a whole 15 s stop rather than instantly.

## Follow-up after the final review (same recordings, ESPectre @20 slots/s)

Seconds of each signal per window:

| Window | Raw ratio ≥ 1 | Debounced motion |
|---|---|---|
| 9 quiet windows (20–40 s each, both links) | 0–2.0 s | **0 s in all 9** |
| Walks, COM15 70–110 / 120–150, COM13 70–110 / 120–145 | 7.5–13.5 s | **4.3–8.8 s** |
| Weak walk stretch, COM15 120–135 | 2.25 s | **0 s** |

- Room evidence now uses only the **debounced** decision for motion. Raw spikes overlap the quiet range
  and would make false "occupied" readings.
- **"Empty" needs 30 s of quiet coverage.** A single 15 s stop can miss a weak walk stretch, so it can
  report a person but never "clear".

Slow (relayed) links, simulated by thinning the 24 pps COM15 link:

| Rate | Setting | Quiet false motion | Walk 70–110 | Walk 120–150 | Ready |
|---|---|---|---|---|---|
| ~8 pps | 10 slots/s, 1 s | 0 s | 4.8 s | 1.5 s | 72–97 % |
| ~8 pps | 10 slots/s, 2 s | 1.8 s | 9.8 s | 9.2 s | 64–99 % |
| ~6 pps | 10 slots/s, 1 s | 0 s | 3.0 s | 0 s | 32–93 % |
| ~6 pps | 5 slots/s, 2 s | 0 s | 9.0 s | 0 s | 97–100 % |

Decision: links at or above 7.5 pps keep 10+ slots/s with a 1 s window; slower links switch to 5 slots/s
with a 2 s window. Slow links still miss weak motion. The robust fix is physical: keep two boards on USB
at the base computer, so only one board pair has to be relayed.

## Live 4-board mesh (2026-10-24 05:50, open 1.70 x 1.20 m area)

The first live run had 7 of 9 one-way links not ready after a minute. Replaying that recording through the
server's `Link` class showed two causes. Rates hovered at slot-rate boundaries, so detectors were rebuilt
(and reset) 5-21 times in 2.5 min. Arrivals are also irregular, so slots near 0.9 x the packet rate stayed
~30 % empty, right at ESPectre's 70 % minimum.

| Rule | Ready (share of seconds) | Rebuilds per link |
|---|---|---|
| old: slots <= 0.9 x rate, slow below 7.5 pps, 2 agreeing checks | 52-91 % | 5-21 |
| new: slots <= 0.6 x rate, slow (5 slots / 2 s) below 15 pps, 10 agreeing checks (5 s) | 94-98 % | 2-7 |

On the old single-link recordings, 10 slots / 1 s (what 16-30 pps direct links now get) was 97-98 % ready,
with 0 s of false motion in the quiet minute and 16-21 s of motion in 3 of the 4 walk windows; it missed
COM15's weaker walk at 120-145 s, which 20 slots / 1 s caught for 3.0 s. 5 slots / 2 s (relayed links) had
1.0 s of false motion in COM15's quiet minute.

Evaluation cadence (same day): links now evaluate on every packet (ESPectre's own 250 ms limit) instead of every
5th packet, which at ~10 pps meant every 0.5 s and a 2 s debounce. Old recordings with the new cadence: 10 slots /
1 s 0 s false motion in both quiet minutes; 5 slots / 2 s 1.2 s (COM15) and 1.5 s (COM13).

## Live 4-phase test (2026-10-24 06:38, boards at chest height, clear surroundings)

60 s each: nobody within 3 m / one person moving inside / moving 0.5-1 m outside the edge / standing still inside.
Share of 0.5 s samples with motion reported:

| Rule | Nobody | Inside, moving | Outside, moving | Inside, still |
|---|---|---|---|---|
| any single link (old) | 5 % | 99 % | 70 % | 6 % |
| at least 2 pairs agree (now) | 0 % | 87 % | 45 % | 0 % |
| at least 3 pairs agree | 0 % | 51 % | 14 % | 0 % |

First detection after walking in: 3 s for all three rules. An earlier run with boards at 20 cm and people
passing behind the area could not tell the phases apart (any link 67-79 % in every phase).

Still person: motion detection does not see it, and breathing-band energy (0.15-0.5 Hz) was no higher than with
nobody there. The GW-BC link through the middle lost 5 dB (-56 to -61 dBm) while the person stood on it
(shadowing); no other link changed by more than 2 dB.

## Wander detector (2026-10-24, replayed on the 06:38 live test)

Snapshot of the previous version: `snapshots/v1-2026-10-24-best-live`.

Under ESPectre with two agreeing pairs, one pair did nearly all the work. The breadcrumb→gateway link fired 96 % of
the time someone walked inside, 77 % for someone outside and 8 % with nobody near. The other eight links fired only
6-36 % of the time with someone walking inside. ESPectre's model was trained on 100 packets/s (augmented down to
70); our links run at 20-44.

Per-link separability (AUC, someone walking inside against nobody / against someone walking outside), 2 s windows:

| Measure | Range over the 9 links |
|---|---|
| ESPectre probability | 0.71-0.95 / 0.58-0.74 |
| wander: 1 - corr(packet shape, 2 s mean shape), esp-radar's measure | 0.53-0.98 / 0.56-0.78 |
| subcarrier CV (std / mean) | 0.58-0.97 / 0.58-0.86 |

Each link's measure is divided by its own 5th percentile over the last 5 minutes. A board pair's score is the mean
of its two directions' log ratios. An alarm needs one pair above 1.8 for 0.5 s. Replayed through the server code:

| Rule | Nobody | Inside, moving | Outside, moving | Inside, still | First hit |
|---|---|---|---|---|---|
| ESPectre, 2 pairs (as run live) | 0 % | 86 % | 48 % | 0 % | 2.9 s |
| ESPectre, 2 pairs, relays on the gateway clock | 0 % | 84 % | 34 % | 0 % | 2.9 s |
| wander, 1 pair > 1.8 (new default) | 0 % | 94 % | 7 % | 0 % | 2.4 s |

The wander threshold was chosen on this same recording. Results are flat from 1.6 (97 % / 14 %) to 2.0 (90 % / 3 %).
The variance-type measures (CV, PCA) were as good at their best threshold but jumped to 4-28 % false alarms 0.1
below it. The 06:17 run (boards at 20 cm, people passing behind) separates nothing with either detector.

Data path: the gateway prints lines up to 250 ms late and in bursts (host-arrival jitter 42 ms std). Only 8 of 188
per-link gaps over 0.2 s were real on the board clock. Relayed lines now take the gateway clock of the CSIA line
printed before them. Halving the 44 pps links to 22 pps changed their wander AUC by at most 0.02, so faster relays
(a firmware change) are not worth a reflash.
