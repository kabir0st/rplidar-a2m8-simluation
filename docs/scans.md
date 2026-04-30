# What a reading contains

Every tick the simulator emits one **sample**. A full **scan** is 360 of those samples laid end-to-end at 1-degree spacing. This doc describes the meaning of a sample once you've decoded it; for the byte-level layout see [protocol.md](protocol.md).

See [`../README.md`](../README.md) for the high-level pitch.

## Per-sample fields

| Field | Type | Meaning |
|-------|------|---------|
| `angle_deg` | float, 0.0–359.984 | Bearing of the ray, measured CCW from the +x axis in **canvas coordinates** (y-down). |
| `distance_mm` | float, 0.0 or 150.0–12000.0 | Range to the nearest obstacle along the ray. `0.0` means "no return" (out of range, blocked, or below the 150 mm dead zone). |
| `quality` | int, 0–63 | Confidence-style byte. `0` = invalid; otherwise lands in the 12–47 band in this simulator. |
| `is_new_scan` | bool | `True` on the first sample of a fresh 360-degree sweep, `False` otherwise. |

## Ranges and limits

The simulator faithfully models the RPLidar A2M8 working envelope (see [`../libs/server.py`](../libs/server.py)):

```
LIDAR_MIN_MM = 150
LIDAR_MAX_MM = 12000
SAMPLE_PERIOD = 0.005  # 200 samples/s
ANGLE_STEP   = 1.0     # degrees
```

That gives:
- ~200 samples/s, ~1.8 full scans/s.
- Returns in the range `[150 mm, 12000 mm]` get reported with their measured (noisy) distance.
- Anything closer than 150 mm or beyond the 12 m max returns `distance_mm = 0.0` and `quality = 0`.

## Noise model

Distances are perturbed with Gaussian noise:

```
sigma     = max(NOISE_FLOOR_MM, NOISE_PROPORTIONAL * true_mm)
noisy_mm  = N(true_mm, sigma)
```

with `NOISE_FLOOR_MM = 10.0` and `NOISE_PROPORTIONAL = 0.01`. So expect roughly 10 mm of noise within the first metre and ~1% of the range further out (e.g. ~50 mm sigma at 5 m). The noisy value is clamped back into `[150, 12000]` mm before being sent.

## Quality model

Quality drops linearly with distance, with a small jitter:

```
base   = QUALITY_MAX - (QUALITY_MAX - QUALITY_MIN) * (distance / LIDAR_MAX_MM)
qual   = round(base + uniform(-2, +2))   # clamped to [QUALITY_MIN, QUALITY_MAX]
```

with `QUALITY_MAX = 47` and `QUALITY_MIN = 12`. Most SLAM front-ends just want a non-zero quality as a "valid" check — anything `> 0` is a real return.

## Assembling a full scan

The protocol is sample-stream-based, not frame-based. To build a 360-degree scan:

1. Keep an empty list `current_scan = []`.
2. For each decoded sample:
   - If `is_new_scan` is True and `current_scan` is non-empty, emit `current_scan` to your algorithm and start a new one.
   - Append `(angle_deg, distance_mm, quality)` to `current_scan`.

`is_new_scan` is set on the sample where the angle wraps past 360 back to 0, so you'll get exactly one `True` per sweep. Expect 360 samples per scan (one per integer degree).

## Coordinate convention

The lidar position is a pixel coordinate inside an 800x600 canvas (Tkinter's coordinate system, y-down). Angle 0 points right (+x), and angles increase clockwise as drawn on screen because of the y-flip. If you're feeding the scan into a SLAM algorithm that assumes a standard math convention (y-up, CCW positive), either negate the angles or flip y after the polar-to-Cartesian conversion. The simulator's ground-truth pose is just `(lidar_x, lidar_y) * MM_PER_PIXEL` (with `MM_PER_PIXEL = 10`) — handy for evaluating estimators.

## Where to look in the code

- Sample generation and noise: [`../libs/server.py`](../libs/server.py) — `_stream_loop` and `_quality_for_distance`.
- Ray-casting: [`../libs/geometry.py`](../libs/geometry.py) — 2D segment-intersection used for every sample.
- Encoding to bytes: [`../libs/protocol.py`](../libs/protocol.py) — see [protocol.md](protocol.md) for the bitfields.
