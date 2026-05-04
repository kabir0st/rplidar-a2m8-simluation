"""Tkinter UI: arena canvas, draggable lidar, click-drag boxes/walls."""

import math
import tkinter as tk

from libs.geometry import cast_ray

LIDAR_RADIUS = 7
RAY_PREVIEW_STEP_DEG = 6      # draw every Nth ray on screen
PREVIEW_REFRESH_MS = 100


class SimUI:
    def __init__(self, root, world, mm_per_pixel):
        self.world = world
        self.mm_per_pixel = mm_per_pixel
        self.mode = tk.StringVar(value="lidar")
        self.coord_frame = tk.StringVar(value="world")

        self._build_toolbar(root)
        self.canvas = tk.Canvas(
            root, width=world.width, height=world.height, bg="white",
            highlightthickness=0,
        )
        self.canvas.pack()

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
            ("Move Lidar", "lidar"),
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
        if self.mode.get() == "lidar":
            self.world.set_lidar(e.x, e.y)
            self._redraw()

    def _on_drag(self, e):
        if self._drag_start is None:
            return
        if self.mode.get() == "lidar":
            self.world.set_lidar(e.x, e.y)
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
        # Obstacles + endpoint labels
        labeled = set()
        for x1, y1, x2, y2 in self.world.obstacles:
            self.canvas.create_line(x1, y1, x2, y2, fill="#444", width=2)
            for px, py in ((x1, y1), (x2, y2)):
                key = (round(px, 1), round(py, 1))
                if key in labeled:
                    continue
                labeled.add(key)
                mx, my = self._to_display_mm(px, py, pose)
                self.canvas.create_text(
                    px, py - 8, text=f"({mx:.0f},{my:.0f})",
                    fill="#666", font=("TkDefaultFont", 8), anchor="s",
                )
        # Lidar rays + dot
        self._draw_rays(pose)

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
