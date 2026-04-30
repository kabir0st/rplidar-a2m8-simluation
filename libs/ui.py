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

    def _redraw(self):
        self.canvas.delete("all")
        # Arena border
        self.canvas.create_rectangle(
            1, 1, self.world.width - 1, self.world.height - 1,
            outline="black", width=2,
        )
        # Obstacles
        for x1, y1, x2, y2 in self.world.obstacles:
            self.canvas.create_line(x1, y1, x2, y2, fill="#444", width=2)
        # Lidar rays + dot
        self._draw_rays()

    def _draw_rays(self):
        lx, ly, segs = self.world.snapshot()
        max_pixels = 12000 / self.mm_per_pixel
        for deg in range(0, 360, RAY_PREVIEW_STEP_DEG):
            ang = math.radians(deg)
            d = cast_ray(lx, ly, ang, segs, max_pixels)
            ex = lx + d * math.cos(ang)
            ey = ly + d * math.sin(ang)
            self.canvas.create_line(lx, ly, ex, ey, fill="#a8d8ff")
        self.canvas.create_oval(
            lx - LIDAR_RADIUS, ly - LIDAR_RADIUS,
            lx + LIDAR_RADIUS, ly + LIDAR_RADIUS,
            fill="#1a73e8", outline="black",
        )
