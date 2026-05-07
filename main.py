"""RPLidar A2M8 simulator with a Tkinter scene editor.

Run this and then point the consumer (lab-2) at 127.0.0.1:9887 (control)
and have it listen on 127.0.0.1:9888 (data) like before.
"""

import tkinter as tk

from libs.server import LidarServer
from libs.ui import SimUI
from libs.world import World

ARENA_W = 800
ARENA_H = 600
MM_PER_PIXEL = 10.0  # 1 px = 10 mm  ->  arena is 8 m x 6 m


def main():
    world = World(ARENA_W, ARENA_H, MM_PER_PIXEL)
    LidarServer(world, MM_PER_PIXEL).start()

    root = tk.Tk()
    root.title("RPLidar A2M8 Simulator")
    root.resizable(False, False)
    SimUI(root, world, MM_PER_PIXEL)
    root.mainloop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[lidar-sim] shutting down")
