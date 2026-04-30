# Wire protocol

The simulator speaks an RPLidar-A2M8-style binary protocol over two TCP sockets on `127.0.0.1`. If you already have an A2M8 driver, the parsing logic carries over with only the transport swapped.

See [scans.md](scans.md) for what to do with a sample once it's decoded, and [`../README.md`](../README.md) for the top-level overview.

## Sockets

| Direction | Host | Port | Role |
|-----------|------|------|------|
| Client connects in | `127.0.0.1` | `9887` | **Control** — send `START` / `STOP` here. |
| Simulator connects out | `127.0.0.1` | `9888` | **Data** — your code must be **listening** on this port. |

The data socket is reversed from a normal client/server — when you send `START`, the simulator opens an outbound TCP connection to your listener. Bind 9888 *before* sending `START`.

## Control commands

The simulator reads exactly 2 bytes per control connection. The first byte is the command; the second byte is currently ignored but must be present.

```
0x10  START   begin streaming
0x20  STOP    halt the streamer
```

Source: [`../libs/protocol.py`](../libs/protocol.py).

## Frame format

Every sample arrives wrapped in a 5-byte header:

```
+------+------+--------+--------+--------+================+
| 0xA5 | 0x10 | sz_hi  | sz_mid | sz_lo  |   payload...   |
+------+------+--------+--------+--------+================+
        sync                size (24-bit big-endian)
```

In this simulator the streamer emits one sample per frame, so `size = 5` and a full frame is 8 bytes on the wire. The format permits batched payloads, so a robust parser should still read `size` bytes after the header rather than assuming 5.

Source: `frame_packet` in [`../libs/protocol.py`](../libs/protocol.py).

## Sample bitfields (5-byte payload)

From `encode_sample`:

```
angle_q6    = int(angle_deg * 64) & 0x7FFF      # 15-bit, Q9.6
distance_q2 = int(distance_mm * 4) & 0xFFFF     # 16-bit, Q14.2
```

| Byte | Bits 7..0 | Meaning |
|------|-----------|---------|
| 0 | `QQQQQQ NS` | bits 7..2 = 6-bit `quality`; bit 1 = `~is_new_scan`; bit 0 = `is_new_scan` |
| 1 | `AAAAAAA 1` | bits 7..1 = `angle_q6` low 7 bits; bit 0 = constant `1` |
| 2 | `AAAAAAAA` | `angle_q6` bits 14..7 |
| 3 | `DDDDDDDD` | `distance_q2` low byte |
| 4 | `DDDDDDDD` | `distance_q2` high byte |

The `~is_new_scan` bit at position 1 of byte 0 is the standard RPLidar invariant `S xor not_S == 1`, useful as a sanity check.

## Decoding a sample

```python
def decode_sample(b):
    quality     = (b[0] >> 2) & 0x3F
    is_new_scan = (b[0] & 0x01) == 1
    angle_q6    = ((b[2] << 7) | (b[1] >> 1)) & 0x7FFF
    angle_deg   = angle_q6 / 64.0
    distance_q2 = (b[4] << 8) | b[3]
    distance_mm = distance_q2 / 4.0
    return angle_deg, distance_mm, quality, is_new_scan
```

Cross-check: each formula is the inverse of the corresponding line in `encode_sample` ([`../libs/protocol.py`](../libs/protocol.py)).

## Reading a frame from the socket

```python
def read_frame(sock):
    header = recv_exactly(sock, 5)
    if header[0] != 0xA5 or header[1] != 0x10:
        raise ValueError(f"bad sync {header[:2].hex()}")
    size = (header[2] << 16) | (header[3] << 8) | header[4]
    payload = recv_exactly(sock, size)
    # one sample per frame in the current streamer, but loop in case that changes
    return [decode_sample(payload[i:i+5]) for i in range(0, size, 5)]
```

`recv_exactly` is your usual loop around `sock.recv` until N bytes have been collected — `recv` can return fewer bytes than asked for on TCP.

## Compatibility notes

- The framing and sample bitfields match the RPLidar A2M8 standard scan response, so any A2M8 parser should drop in with only the transport swapped (TCP socket instead of serial port).
- Express-mode and other A-series extensions are **not** implemented — the simulator only emits standard scan samples.
- There's no health/info command — the control socket only handles `START` / `STOP`.
