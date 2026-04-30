"""Control + data sockets that mimic the RPLidar A2M8 over TCP.

The control listener handles START/STOP commands. While running, the streamer
walks 360 degrees (1 deg/sample) ray-casting against the World and emits one
A2M8-formatted sample per tick.
"""

import math
import random
import socket
import threading
import time

from libs.geometry import cast_ray
from libs.protocol import CMD_START, CMD_STOP, encode_sample, frame_packet

CONTROL_HOST = "127.0.0.1"
CONTROL_PORT = 9887
DATA_HOST = "127.0.0.1"
DATA_PORT = 9888

SAMPLE_PERIOD = 0.005   # ~200 samples/s -> ~1.8 scans/s at 1 deg step
ANGLE_STEP = 1.0
LIDAR_MIN_MM = 150       # A2M8 minimum range
LIDAR_MAX_MM = 12000     # A2M8 maximum range

# A2M8-ish noise model: floor at close range, +1% of distance further out.
NOISE_FLOOR_MM = 10.0
NOISE_PROPORTIONAL = 0.01
# 6-bit quality byte (0..63). Indoor surfaces typically land in this band.
QUALITY_MAX = 47
QUALITY_MIN = 12


def _quality_for_distance(distance_mm, max_mm):
    t = max(0.0, min(1.0, distance_mm / max_mm))
    base = QUALITY_MAX - (QUALITY_MAX - QUALITY_MIN) * t
    jitter = random.uniform(-2.0, 2.0)
    return max(QUALITY_MIN, min(QUALITY_MAX, int(round(base + jitter))))


class LidarServer:
    def __init__(self, world, mm_per_pixel):
        self.world = world
        self.mm_per_pixel = mm_per_pixel
        self._stop_event = threading.Event()
        self._streamer = None

    def start(self):
        threading.Thread(target=self._control_loop, daemon=True).start()

    def _control_loop(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((CONTROL_HOST, CONTROL_PORT))
        listener.listen(1)
        print(f"[lidar-sim] control ready on {CONTROL_HOST}:{CONTROL_PORT}")

        while True:
            client, _ = listener.accept()
            try:
                cmd = client.recv(2)
            finally:
                client.close()
            if len(cmd) < 2:
                continue

            if cmd[0] == CMD_START:
                print("[lidar-sim] START")
                if self._streamer is None or not self._streamer.is_alive():
                    self._stop_event.clear()
                    self._streamer = threading.Thread(
                        target=self._stream_loop, daemon=True
                    )
                    self._streamer.start()
            elif cmd[0] == CMD_STOP:
                print("[lidar-sim] STOP")
                self._stop_event.set()
            else:
                print(f"[lidar-sim] unknown command 0x{cmd[0]:02X}")

    def _connect_data(self):
        for _ in range(5):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                sock.connect((DATA_HOST, DATA_PORT))
                return sock
            except OSError:
                sock.close()
                time.sleep(0.5)
        return None

    def _stream_loop(self):
        sock = self._connect_data()
        if sock is None:
            print(f"[lidar-sim] could not reach data port {DATA_HOST}:{DATA_PORT}")
            return
        print(f"[lidar-sim] streaming to {DATA_HOST}:{DATA_PORT}")

        max_pixels = LIDAR_MAX_MM / self.mm_per_pixel
        angle = 0.0
        try:
            while not self._stop_event.is_set():
                prev_angle = angle
                angle += ANGLE_STEP
                is_new_scan = angle >= 360.0
                if is_new_scan:
                    angle -= 360.0

                lx, ly, segs = self.world.snapshot()
                ang_rad = math.radians(prev_angle)
                dist_pix = cast_ray(lx, ly, ang_rad, segs, max_pixels)
                true_mm = dist_pix * self.mm_per_pixel

                if dist_pix >= max_pixels or true_mm < LIDAR_MIN_MM:
                    dist_mm = 0.0
                    quality = 0
                else:
                    sigma = max(NOISE_FLOOR_MM, NOISE_PROPORTIONAL * true_mm)
                    noisy = random.gauss(true_mm, sigma)
                    dist_mm = max(
                        float(LIDAR_MIN_MM), min(float(LIDAR_MAX_MM), noisy)
                    )
                    quality = _quality_for_distance(dist_mm, LIDAR_MAX_MM)

                packet = frame_packet(
                    encode_sample(quality, prev_angle, dist_mm, is_new_scan)
                )
                sock.sendall(packet)
                time.sleep(SAMPLE_PERIOD)
        except OSError as e:
            print(f"[lidar-sim] stream ended: {e}")
        finally:
            try:
                sock.close()
            except OSError:
                pass
            print("[lidar-sim] streamer stopped")
