"""Shared scene model: arena bounds, lidar pose, and obstacle line segments.

Everything is stored in canvas pixel coordinates. The server converts to mm
using a `mm_per_pixel` scale when it needs real-world distances.
"""

import threading


class World:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.lidar_x = width / 2
        self.lidar_y = height / 2
        # Each obstacle is a tuple (x1, y1, x2, y2). Boxes are stored as 4 segs.
        self.obstacles = []
        self._lock = threading.Lock()

    def _arena_segments(self):
        w, h = self.width, self.height
        return [
            (0, 0, w, 0),
            (w, 0, w, h),
            (w, h, 0, h),
            (0, h, 0, 0),
        ]

    def add_box(self, x1, y1, x2, y2):
        with self._lock:
            self.obstacles.extend([
                (x1, y1, x2, y1),
                (x2, y1, x2, y2),
                (x2, y2, x1, y2),
                (x1, y2, x1, y1),
            ])

    def add_wall(self, x1, y1, x2, y2):
        with self._lock:
            self.obstacles.append((x1, y1, x2, y2))

    def clear(self):
        with self._lock:
            self.obstacles.clear()

    def set_lidar(self, x, y):
        with self._lock:
            self.lidar_x = max(1, min(self.width - 1, x))
            self.lidar_y = max(1, min(self.height - 1, y))

    def snapshot(self):
        """Return a thread-safe copy of (lidar_x, lidar_y, all_segments)."""
        with self._lock:
            return (
                self.lidar_x,
                self.lidar_y,
                self._arena_segments() + list(self.obstacles),
            )
