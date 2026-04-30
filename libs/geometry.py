"""2D ray-casting helpers used by both the simulator and the UI preview."""

import math


def ray_segment_intersection(ox, oy, dx, dy, x1, y1, x2, y2):
    """Distance along ray (origin + t*dir) where it hits segment, or None."""
    sx = x2 - x1
    sy = y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if t >= 0 and 0.0 <= u <= 1.0:
        return t
    return None


def cast_ray(ox, oy, angle_rad, segments, max_dist):
    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)
    closest = max_dist
    for seg in segments:
        t = ray_segment_intersection(ox, oy, dx, dy, *seg)
        if t is not None and t < closest:
            closest = t
    return closest
