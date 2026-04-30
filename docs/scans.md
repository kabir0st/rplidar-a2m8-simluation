# What a reading contains

Every tick the simulator emits one **sample**. A full **scan** is roughly 180 of those samples spread across 360 degrees at a nominal 2-degree step (with occasional repeats and 4-/6-degree skips, mirroring the real device). This doc describes the meaning of a sample once you've decoded it; for the byte-level layout see [protocol.md](protocol.md).

See [`../README.md`](../README.md) for the high-level pitch.

## Per-sample fields

| Field | Type | Meaning |
|-------|------|---------|
| `angle_deg` | int (even), 0–358 | Bearing of the ray, measured CCW from the +x axis in **canvas coordinates** (y-down). Emitted at 2-degree resolution to match the real device. |
| `distance_mm` | int, 0 or 150–12000 | Range to the nearest obstacle along the ray. `0` means "no return" (out of range, blocked, below the 150 mm dead zone, or a random dropout). |
| `quality` | int, 0–63 | Confidence-style byte. `0` = invalid; valid returns are almost always `15`, with a thin tail at 11–14 and 22, matching real captures. |
| `is_new_scan` | bool | `True` on the first sample of a fresh 360-degree sweep, `False` otherwise. |

## Ranges and limits

The simulator faithfully models the RPLidar A2M8 working envelope (see [`../libs/server.py`](../libs/server.py)):

```
LIDAR_MIN_MM   = 150
LIDAR_MAX_MM   = 12000
SAMPLE_PERIOD  = 0.003   # ~330 samples/s
ANGLE_STEP_CHOICES = (0, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 6)
```

That gives:
- ~330 samples/s, ~1.8 full scans/s (~180 samples per scan).
- Angle deltas drawn from the empirical distribution: ~82% +2°, ~12% +4°, ~4% repeats (0°), ~2% +6°.
- Returns in the range `[150 mm, 12000 mm]` get reported with their measured (noisy, integer-rounded) distance.
- Anything closer than 150 mm or beyond the 12 m max returns `distance_mm = 0` and `quality = 0`.
- Even within a valid arc, ~30% of would-be returns drop out with `distance_mm = 0` while keeping a non-zero quality — same intermittent-zero pattern visible in real captures.

## Noise model

Distances are perturbed with Gaussian noise:

```
sigma     = max(NOISE_FLOOR_MM, NOISE_PROPORTIONAL * true_mm)
noisy_mm  = N(true_mm, sigma)
```

with `NOISE_FLOOR_MM = 10.0` and `NOISE_PROPORTIONAL = 0.01`. So expect roughly 10 mm of noise within the first metre and ~1% of the range further out (e.g. ~50 mm sigma at 5 m). The noisy value is clamped back into `[150, 12000]` mm before being sent.

## Quality model

Quality is a narrow band centered on `15`, matching what the A2M8 actually emits in practice:

```
QUALITY_TYPICAL  = 15
QUALITY_TAIL     = (11, 12, 13, 14, 22)
QUALITY_TAIL_PROB = 0.01     # 1% of valid returns land in the tail
```

So ~99% of valid returns carry quality `15`, the rest land in the small tail. Most SLAM front-ends just want a non-zero quality as a "valid" check, anything `> 0` is a real return.

## Assembling a full scan

The protocol is sample-stream-based, not frame-based. To build a 360-degree scan:

1. Keep an empty list `current_scan = []`.
2. For each decoded sample:
   - If `is_new_scan` is True and `current_scan` is non-empty, emit `current_scan` to your algorithm and start a new one.
   - Append `(angle_deg, distance_mm, quality)` to `current_scan`.

`is_new_scan` is set on the sample where the angle wraps past 360 back to 0, so you'll get exactly one `True` per sweep. Expect 360 samples per scan (one per integer degree).

## Coordinate convention

The lidar position is a pixel coordinate inside an 800x600 canvas (Tkinter's coordinate system, y-down). Angle 0 points right (+x), and angles increase clockwise as drawn on screen because of the y-flip. If you're feeding the scan into a SLAM algorithm that assumes a standard math convention (y-up, CCW positive), either negate the angles or flip y after the polar-to-Cartesian conversion. The simulator's ground-truth pose is just `(lidar_x, lidar_y) * MM_PER_PIXEL` (with `MM_PER_PIXEL = 10`), handy for evaluating estimators.

## Where to look in the code

- Sample generation and noise: [`../libs/server.py`](../libs/server.py), `_stream_loop` and `_quality_for_distance`.
- Ray-casting: [`../libs/geometry.py`](../libs/geometry.py), 2D segment-intersection used for every sample.
- Encoding to bytes: [`../libs/protocol.py`](../libs/protocol.py), see [protocol.md](protocol.md) for the bitfields.
