# Integrating with SLAM / scan-matching code

This is the doc for the situation that motivated the project: you're writing a 2D scan-based algorithm (scan_cox, ICP, Hector-style scan matching, particle-filter SLAM, etc.) and you want fresh, parameter-controllable scans without a real lidar in the loop.

See [`../README.md`](../README.md) for the overview, [protocol.md](protocol.md) for the byte-level details, and [scans.md](scans.md) for what's in a decoded sample.

## Workflow

1. Run `python main.py` in one terminal.
2. Use the GUI to lay out the scene you want, drag the lidar, paint walls/boxes, clear and re-paint between trials.
3. From your algorithm: bind a listener on `127.0.0.1:9888`, then send `START` (`0x10 0x00`) to `127.0.0.1:9887`. The simulator dials in to your listener and starts streaming.
4. Decode frames per [protocol.md](protocol.md), accumulate samples until `is_new_scan` per [scans.md](scans.md), and feed each completed scan into your algorithm.
5. Send `STOP` (`0x20 0x00`) when done.

## Reference client

Drop-in replacement for an RPLidar driver. Yields complete 360-sample scans as `(angle_deg, distance_mm, quality)` lists.

```python
import socket

CTRL = ("127.0.0.1", 9887)
DATA = ("127.0.0.1", 9888)


def recv_exactly(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("stream closed")
        buf.extend(chunk)
    return bytes(buf)


def decode_sample(b):
    quality     = (b[0] >> 2) & 0x3F
    is_new_scan = (b[0] & 0x01) == 1
    angle_deg   = (((b[2] << 7) | (b[1] >> 1)) & 0x7FFF) / 64.0
    distance_mm = ((b[4] << 8) | b[3]) / 4.0
    return angle_deg, distance_mm, quality, is_new_scan


def scans():
    listener = socket.socket()
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(DATA)
    listener.listen(1)

    ctrl = socket.socket()
    ctrl.connect(CTRL)
    ctrl.send(bytes([0x10, 0x00]))   # START
    ctrl.close()

    stream, _ = listener.accept()
    listener.close()

    current = []
    try:
        while True:
            header = recv_exactly(stream, 5)
            assert header[0] == 0xA5 and header[1] == 0x10, "bad sync"
            size = (header[2] << 16) | (header[3] << 8) | header[4]
            payload = recv_exactly(stream, size)
            for i in range(0, size, 5):
                angle, dist, qual, is_new = decode_sample(payload[i:i+5])
                if is_new and current:
                    yield current
                    current = []
                current.append((angle, dist, qual))
    finally:
        stream.close()


if __name__ == "__main__":
    for scan in scans():
        # Hand the scan off to your algorithm here.
        print(f"scan: {len(scan)} samples, "
              f"{sum(1 for _, d, _ in scan if d > 0)} valid returns")
```

## Plugging into a SLAM front-end

Most 2D SLAM stacks want scans as a list of `(angle_rad, range_m)` pairs in the sensor frame. From the tuple above:

```python
import math

def to_polar_meters(scan):
    return [
        (math.radians(a), d / 1000.0)
        for a, d, q in scan
        if q > 0 and d > 0.0
    ]
```

Filter on `quality > 0` to drop the "no return" samples. If your algorithm assumes y-up / CCW angles, also negate the angle (the simulator runs in canvas coordinates, y-down; see [scans.md](scans.md#coordinate-convention)).

## Testing patterns

- **Loop closure stress test**, use Walls mode to draw a closed corridor that returns to the start. Move the lidar around the loop between scans (each move = a fake odometry step) and watch whether your algorithm closes the loop.
- **Feature-poor scenes**, clear all obstacles. Pure rectangular arena will starve most scan matchers; useful for finding the failure mode.
- **Feature-rich scenes**, pile boxes of varying sizes around the lidar. A clean baseline for ICP convergence.
- **Range edge cases**, drop a wall right next to the lidar (under the 150 mm dead zone) or far enough away (>12 m would need a bigger arena, but 12 m max is the practical edge) to see how your code handles `distance_mm == 0` markers.
- **Ground truth**, the simulator's "true" sensor pose is just `(lidar_x, lidar_y) * 10 mm`. Read it off the canvas, compare to whatever pose your SLAM estimator produces.

## Caveats

- The simulator is 2D only. There is no roll/pitch/elevation channel.
- Scan rate is fixed at the parameters in [`../libs/server.py`](../libs/server.py); change the constants there if you need a different cadence.
- The lidar pose is updated through the GUI, not the network, there's no API for an external program to teleport it. If your test loop needs that, automate the GUI or extend `World` with a network setter.
- One client at a time. The data port accepts a single inbound connection per `START`.
