"""Tkinter UI: arena canvas, draggable robot, click-drag boxes/walls, motor controls."""

import math
import tkinter as tk

from libs.geometry import cast_ray
from libs.robot import (
    CHASSIS_LENGTH_MM,
    CHASSIS_WIDTH_MM,
    LIDAR_OFFSET_MM,
    WHEEL_DIAMETER_MM,
    WHEELBASE_MM,
)

LIDAR_RADIUS = 7
RAY_PREVIEW_STEP_DEG = 6      # draw every Nth ray on screen
PREVIEW_REFRESH_MS = 100

MOTOR_STEP = 10               # Set_Speed units per +/- click


class SimUI:
    def __init__(self, root, world, mm_per_pixel):
        self.world = world
        self.mm_per_pixel = mm_per_pixel
        self.mode = tk.StringVar(value="robot")
        self.coord_frame = tk.StringVar(value="world")
        self.motors_linked = False
        self.set_speed_l = tk.IntVar(value=0)
        self.set_speed_r = tk.IntVar(value=0)

        self._build_toolbar(root)
        self._build_motor_bar(root)
        self.canvas = tk.Canvas(
            root, width=world.width, height=world.height, bg="white",
            highlightthickness=0,
        )
        self.canvas.pack()

        self.status_var = tk.StringVar(
            value="Lidar global: x=+0.0 mm  y=+0.0 mm  θ=+0.00°"
        )
        tk.Label(
            root, textvariable=self.status_var, anchor="w",
            font=("TkFixedFont", 10), relief=tk.SUNKEN, padx=6,
        ).pack(side=tk.TOP, fill=tk.X)

        self.canvas.bind("<Button-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)

        self._drag_start = None
        self._preview_id = None

        self._redraw()
        root.after(PREVIEW_REFRESH_MS, self._tick)

    def _build_toolbar(self, root):
        bar = tk.Frame(root)
        bar.pack(side=tk.TOP, fill=tk.X)
        modes = [
            ("Move Robot", "robot"),
            ("Add Box", "box"),
            ("Build Wall", "wall"),
        ]
        for label, val in modes:
            tk.Radiobutton(bar, text=label, variable=self.mode, value=val).pack(
                side=tk.LEFT
            )
        tk.Button(bar, text="Clear obstacles", command=self._on_clear).pack(
            side=tk.LEFT, padx=8
        )
        tk.Label(bar, text="Heading °").pack(side=tk.LEFT)
        self.heading_scale = tk.Scale(
            bar, from_=0, to=359, orient=tk.HORIZONTAL, length=200,
            resolution=1, showvalue=True, command=self._on_heading,
        )
        self.heading_scale.pack(side=tk.LEFT)

        tk.Label(bar, text="Noise σ (mm)").pack(side=tk.LEFT)
        self.sigma_scale = tk.Scale(
            bar, from_=0, to=100, orient=tk.HORIZONTAL, length=200,
            resolution=1, showvalue=True, command=self._on_sigma,
        )
        self.sigma_scale.set(int(self.world.noise_sigma_mm))
        self.sigma_scale.pack(side=tk.LEFT)

        self.frame_button = tk.Button(
            bar, text="Frame: World", command=self._on_toggle_frame,
        )
        self.frame_button.pack(side=tk.LEFT, padx=8)

    def _build_motor_bar(self, root):
        bar = tk.Frame(root)
        bar.pack(side=tk.TOP, fill=tk.X, pady=(2, 4))

        tk.Label(bar, text="Motor L").pack(side=tk.LEFT, padx=(4, 2))
        tk.Button(bar, text="−", width=2,
                  command=lambda: self._bump_motor("L", -MOTOR_STEP)).pack(side=tk.LEFT)
        tk.Label(bar, textvariable=self.set_speed_l, width=6,
                 relief=tk.SUNKEN, anchor="e").pack(side=tk.LEFT, padx=2)
        tk.Button(bar, text="+", width=2,
                  command=lambda: self._bump_motor("L", +MOTOR_STEP)).pack(side=tk.LEFT)

        self.link_button = tk.Button(
            bar, text="Link: OFF", width=10, command=self._on_toggle_link,
        )
        self.link_button.pack(side=tk.LEFT, padx=12)

        tk.Label(bar, text="Motor R").pack(side=tk.LEFT, padx=(4, 2))
        tk.Button(bar, text="−", width=2,
                  command=lambda: self._bump_motor("R", -MOTOR_STEP)).pack(side=tk.LEFT)
        tk.Label(bar, textvariable=self.set_speed_r, width=6,
                 relief=tk.SUNKEN, anchor="e").pack(side=tk.LEFT, padx=2)
        tk.Button(bar, text="+", width=2,
                  command=lambda: self._bump_motor("R", +MOTOR_STEP)).pack(side=tk.LEFT)

        tk.Button(bar, text="Stop", width=6, command=self._on_stop).pack(
            side=tk.LEFT, padx=12
        )

    def _bump_motor(self, which, delta):
        l = self.set_speed_l.get()
        r = self.set_speed_r.get()
        if self.motors_linked:
            l += delta
            r += delta
        elif which == "L":
            l += delta
        else:
            r += delta
        self.set_speed_l.set(l)
        self.set_speed_r.set(r)
        self.world.set_motor_speeds(l, r)

    def _on_toggle_link(self):
        self.motors_linked = not self.motors_linked
        self.link_button.config(
            text=f"Link: {'ON' if self.motors_linked else 'OFF'}"
        )

    def _on_stop(self):
        self.set_speed_l.set(0)
        self.set_speed_r.set(0)
        self.world.stop_motors()

    def _on_toggle_frame(self):
        new_frame = "lidar" if self.coord_frame.get() == "world" else "world"
        self.coord_frame.set(new_frame)
        self.frame_button.config(text=f"Frame: {new_frame.capitalize()}")
        self._redraw()

    def _on_heading(self, value):
        self.world.set_heading(float(value))
        self._redraw()

    def _on_sigma(self, value):
        self.world.set_noise_sigma(float(value))

    def _on_clear(self):
        self.world.clear()
        self._redraw()

    def _on_press(self, e):
        self._drag_start = (e.x, e.y)
        if self.mode.get() == "robot":
            self.world.set_robot_pose(e.x, e.y)
            self._redraw()

    def _on_drag(self, e):
        if self._drag_start is None:
            return
        if self.mode.get() == "robot":
            self.world.set_robot_pose(e.x, e.y)
            self._redraw()
            return

        if self._preview_id is not None:
            self.canvas.delete(self._preview_id)

        x1, y1 = self._drag_start
        if self.mode.get() == "box":
            self._preview_id = self.canvas.create_rectangle(
                x1, y1, e.x, e.y, outline="red", dash=(3, 2)
            )
        else:  # wall
            self._preview_id = self.canvas.create_line(
                x1, y1, e.x, e.y, fill="red", dash=(3, 2)
            )

    def _on_release(self, e):
        if self._drag_start is None:
            return
        x1, y1 = self._drag_start
        self._drag_start = None
        if self._preview_id is not None:
            self.canvas.delete(self._preview_id)
            self._preview_id = None

        m = self.mode.get()
        if m == "box" and abs(e.x - x1) > 2 and abs(e.y - y1) > 2:
            self.world.add_box(
                min(x1, e.x), min(y1, e.y), max(x1, e.x), max(y1, e.y)
            )
        elif m == "wall" and (abs(e.x - x1) > 2 or abs(e.y - y1) > 2):
            self.world.add_wall(x1, y1, e.x, e.y)
        self._redraw()

    def _tick(self):
        # Skip the periodic refresh while the user is mid-drag for box/wall,
        # otherwise the dashed preview gets wiped.
        drawing_shape = self._drag_start is not None and self.mode.get() in (
            "box", "wall",
        )
        if not drawing_shape:
            self._redraw()
        self.canvas.after(PREVIEW_REFRESH_MS, self._tick)

    def _update_status(self):
        x, y, th = self.world.get_global_pose()
        self.status_var.set(
            f"Lidar global: x={x:+8.1f} mm  y={y:+8.1f} mm  θ={th:+7.2f}°"
        )

    def _to_display_mm(self, px, py, lidar_pose):
        lx, ly, heading_deg, *_ = lidar_pose
        if self.coord_frame.get() == "lidar":
            dx = (px - lx) * self.mm_per_pixel
            dy = (py - ly) * self.mm_per_pixel
            ang = -math.radians(heading_deg)
            x = dx * math.cos(ang) - dy * math.sin(ang)
            y = dx * math.sin(ang) + dy * math.cos(ang)
            return x, y
        return px * self.mm_per_pixel, py * self.mm_per_pixel

    def _redraw(self):
        self.canvas.delete("all")
        pose = self.world.snapshot()
        # Arena border
        self.canvas.create_rectangle(
            1, 1, self.world.width - 1, self.world.height - 1,
            outline="black", width=2,
        )
        # Obstacles + endpoint labels (labels go above lidar-y, below lidar-y
        # so they sit clear of the ray fan).
        lidar_y = pose[1]
        labeled = set()
        for x1, y1, x2, y2 in self.world.obstacles:
            self.canvas.create_line(x1, y1, x2, y2, fill="#444", width=2)
            for px, py in ((x1, y1), (x2, y2)):
                key = (round(px, 1), round(py, 1))
                if key in labeled:
                    continue
                labeled.add(key)
                mx, my = self._to_display_mm(px, py, pose)
                if py < lidar_y:
                    text_y, anchor = py - 8, "s"
                else:
                    text_y, anchor = py + 8, "n"
                self.canvas.create_text(
                    px, text_y, text=f"({mx:.0f},{my:.0f})",
                    fill="#666", font=("TkDefaultFont", 8), anchor=anchor,
                )
        # Robot chassis + wheels (under the rays so the lidar dot stays on top)
        self._draw_robot()
        # Lidar rays + dot
        self._draw_rays(pose)
        self._update_status()

    def _local_to_canvas(self, cx_px, cy_px, theta, x_mm, y_mm):
        """Transform a point in robot-local mm coords to canvas pixels."""
        x_px = x_mm / self.mm_per_pixel
        y_px = y_mm / self.mm_per_pixel
        rx = x_px * math.cos(theta) - y_px * math.sin(theta)
        ry = x_px * math.sin(theta) + y_px * math.cos(theta)
        return cx_px + rx, cy_px + ry

    def _draw_robot(self):
        snap = self.world.robot.get_chassis_snapshot()
        cx, cy, theta = snap["x_px"], snap["y_px"], snap["theta_rad"]

        # Chassis body: from x=-30mm (rear overhang) to x=CHASSIS_LENGTH_MM,
        # full width = CHASSIS_WIDTH_MM. Local y axis = robot's right side.
        rear_overhang = 30
        half_w = CHASSIS_WIDTH_MM / 2.0
        corners_local = [
            (-rear_overhang, -half_w),
            (CHASSIS_LENGTH_MM, -half_w),
            (CHASSIS_LENGTH_MM, +half_w),
            (-rear_overhang, +half_w),
        ]
        body_pts = []
        for x_mm, y_mm in corners_local:
            px, py = self._local_to_canvas(cx, cy, theta, x_mm, y_mm)
            body_pts.extend([px, py])
        self.canvas.create_polygon(
            body_pts, fill="#e0ecff", outline="#1a73e8", width=2,
        )

        # Rear driven wheels: at the rear axle (x=0), offset ±WHEELBASE/2
        # along the local-y axis. Drawn as thin rectangles aligned with the
        # heading direction (long axis = wheel diameter).
        wheel_len = WHEEL_DIAMETER_MM
        wheel_width = 22
        for side_y in (-WHEELBASE_MM / 2.0, +WHEELBASE_MM / 2.0):
            wheel_corners = [
                (-wheel_len / 2.0, side_y - wheel_width / 2.0),
                (+wheel_len / 2.0, side_y - wheel_width / 2.0),
                (+wheel_len / 2.0, side_y + wheel_width / 2.0),
                (-wheel_len / 2.0, side_y + wheel_width / 2.0),
            ]
            pts = []
            for x_mm, y_mm in wheel_corners:
                px, py = self._local_to_canvas(cx, cy, theta, x_mm, y_mm)
                pts.extend([px, py])
            self.canvas.create_polygon(pts, fill="black", outline="black")

        # Front caster: small empty circle at (CHASSIS_LENGTH, 0).
        caster_diam_mm = 40
        caster_x_px, caster_y_px = self._local_to_canvas(
            cx, cy, theta, CHASSIS_LENGTH_MM, 0.0,
        )
        r = (caster_diam_mm / 2.0) / self.mm_per_pixel
        self.canvas.create_oval(
            caster_x_px - r, caster_y_px - r,
            caster_x_px + r, caster_y_px + r,
            fill="white", outline="black", width=2,
        )

        # Lidar mount marker: small grey dot under the blue lidar dot, at
        # the lidar offset, so you can see the mounting point even without
        # rays drawn.
        lidar_x_px, lidar_y_px = self._local_to_canvas(
            cx, cy, theta, LIDAR_OFFSET_MM, 0.0,
        )
        self.canvas.create_oval(
            lidar_x_px - 2, lidar_y_px - 2,
            lidar_x_px + 2, lidar_y_px + 2,
            fill="#888", outline="",
        )

    def _draw_rays(self, pose):
        lx, ly, heading_deg, _sigma, segs = pose
        max_pixels = 12000 / self.mm_per_pixel
        for deg in range(0, 360, RAY_PREVIEW_STEP_DEG):
            ang = math.radians(deg + heading_deg)
            d = cast_ray(lx, ly, ang, segs, max_pixels)
            ex = lx + d * math.cos(ang)
            ey = ly + d * math.sin(ang)
            self.canvas.create_line(lx, ly, ex, ey, fill="#a8d8ff")

        # 0° heading marker: red arrow showing which side of the device is 0°.
        zero_ang = math.radians(heading_deg)
        marker_len = 40
        zx = lx + marker_len * math.cos(zero_ang)
        zy = ly + marker_len * math.sin(zero_ang)
        self.canvas.create_line(
            lx, ly, zx, zy, fill="red", width=3, arrow=tk.LAST,
        )
        self.canvas.create_text(
            lx + (marker_len + 12) * math.cos(zero_ang),
            ly + (marker_len + 12) * math.sin(zero_ang),
            text="0°", fill="red", font=("TkDefaultFont", 9, "bold"),
        )

        self.canvas.create_oval(
            lx - LIDAR_RADIUS, ly - LIDAR_RADIUS,
            lx + LIDAR_RADIUS, ly + LIDAR_RADIUS,
            fill="#1a73e8", outline="black",
        )
