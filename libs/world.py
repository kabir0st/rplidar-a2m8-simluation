"""Shared scene model: arena bounds, robot/lidar pose, and obstacle line segments.

Everything is stored in canvas pixel coordinates. The server converts to mm
using a `mm_per_pixel` scale when it needs real-world distances. The lidar
pose is owned by an embedded `Robot` whose kinematics tick in their own
thread, fed by motor commands sent via `set_motor_speeds` / `set_motor_pwm`.
"""

import math
import threading

from libs.robot import Robot


class World:
    def __init__(self, width, height, mm_per_pixel):
        self.width = width
        self.height = height
        self.mm_per_pixel = mm_per_pixel
        self.noise_sigma_mm = 10.0
        # Each obstacle is (x1, y1, x2, y2). Boxes are stored as 4 segs.
        self.obstacles = []
        self._lock = threading.Lock()

        self.robot = Robot(width, height, mm_per_pixel)
        self.robot.start_ticker()

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

    def set_robot_pose(self, x, y):
        """Teleport the robot's rear-axle midpoint (used by drag-to-move UI)."""
        self.robot.set_pose(x_px=x, y_px=y)

    def set_heading(self, deg):
        self.robot.set_pose(theta_rad=math.radians(float(deg)))

    def set_noise_sigma(self, mm):
        with self._lock:
            self.noise_sigma_mm = max(0.0, float(mm))

    def set_motor_speeds(self, left, right):
        """Closed-loop setpoints (Set_Speed_M0/M1 from spi_com.h)."""
        self.robot.set_motor_speeds(left, right)

    def set_motor_pwm(self, left, right):
        """Open-loop PWM commands (Set_PWM_M0/M1 from spi_com.h)."""
        self.robot.set_motor_pwm(left, right)

    def stop_motors(self):
        self.robot.stop()

    def get_global_pose(self):
        """Lidar (x_mm, y_mm, theta_deg) relative to start position."""
        return self.robot.get_global_pose()

    def snapshot(self):
        """Return a thread-safe copy of (lidar_x, lidar_y, heading_deg, noise_sigma_mm, all_segments)."""
        lidar_x, lidar_y, heading_deg = self.robot.get_lidar_pose()
        with self._lock:
            return (
                lidar_x,
                lidar_y,
                heading_deg,
                self.noise_sigma_mm,
                self._arena_segments() + list(self.obstacles),
            )
