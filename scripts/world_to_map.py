#!/usr/bin/env python3
"""
world_to_map.py — Gazebo SDF World -> Nav2 occupancy-grid map
=============================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Inverse of scripts/map_to_world.py. Rasterises every <model name='wall_*'>
box in a Gazebo .sdf world into a 2D occupancy grid (map.pgm + map.yaml) that
Nav2 / AMCL can load. The resulting map frame is aligned 1:1 with the Gazebo
world frame (origin = world min corner), so a robot spawned at world (x, y)
localises at map (x, y) — no manual offset needed.

Use this AT HOME only, to get a map that matches worlds/lab_world.sdf for the
Gazebo Nav2 demo. At the LAB you still produce ~/map.yaml from real-robot SLAM
(README B3); the navigator code path is identical either way.

Requirements: pip3 install pillow numpy   (no ROS needed)

Usage:
  cd ~/turtlebot3_ws/src/farm_twin_poc
  python3 scripts/world_to_map.py worlds/lab_world.sdf ~/map
  # -> writes ~/map.pgm and ~/map.yaml
  ros2 launch farm_twin_poc gazebo_nav2_demo.launch.py map:=$HOME/map.yaml
"""
import os
import re
import sys

import numpy as np
from PIL import Image

RES = 0.05          # m/pixel — matches map_to_world.py wall thickness
MARGIN_CELLS = 4    # free-space border around the walls' bounding box

# Match: <model name='wall_12'> ... <pose>X Y Z r p y</pose> ...
#        <box><size>SX SY SZ</size></box>  (first size = collision box)
MODEL_RE = re.compile(
    r"<model\s+name=['\"](wall_\d+)['\"]>(.*?)</model>", re.DOTALL)
POSE_RE = re.compile(r"<pose>\s*([-\d.eE]+)\s+([-\d.eE]+)", re.DOTALL)
SIZE_RE = re.compile(r"<box><size>\s*([-\d.eE]+)\s+([-\d.eE]+)", re.DOTALL)


def parse_walls(sdf_path):
    with open(sdf_path) as f:
        text = f.read()
    walls = []
    for _name, body in MODEL_RE.findall(text):
        pose = POSE_RE.search(body)
        size = SIZE_RE.search(body)
        if not pose or not size:
            continue
        cx, cy = float(pose.group(1)), float(pose.group(2))
        sx, sy = float(size.group(1)), float(size.group(2))
        walls.append((cx, cy, sx, sy))
    return walls


def rasterise(walls):
    # Bounding box over all wall extents (+ margin so the room isn't flush).
    min_x = min(cx - sx / 2 for cx, cy, sx, sy in walls) - MARGIN_CELLS * RES
    max_x = max(cx + sx / 2 for cx, cy, sx, sy in walls) + MARGIN_CELLS * RES
    min_y = min(cy - sy / 2 for cx, cy, sx, sy in walls) - MARGIN_CELLS * RES
    max_y = max(cy + sy / 2 for cx, cy, sx, sy in walls) + MARGIN_CELLS * RES

    W = int(round((max_x - min_x) / RES))
    H = int(round((max_y - min_y) / RES))

    # 255 = free (white), 0 = occupied (black).  Nav2 default thresholds.
    grid = np.full((H, W), 255, dtype=np.uint8)

    for cx, cy, sx, sy in walls:
        # Wall box -> pixel column/row ranges. Row 0 is the TOP (max_y), so y
        # is flipped; this matches map_to_world.py and Nav2's image convention.
        c0 = int(np.floor((cx - sx / 2 - min_x) / RES))
        c1 = int(np.ceil((cx + sx / 2 - min_x) / RES))
        r0 = int(np.floor((max_y - (cy + sy / 2)) / RES))
        r1 = int(np.ceil((max_y - (cy - sy / 2)) / RES))
        c0, c1 = max(0, c0), min(W, c1)
        r0, r1 = max(0, r0), min(H, r1)
        grid[r0:r1, c0:c1] = 0

    return grid, (min_x, min_y)


def main():
    if len(sys.argv) != 3:
        print('Usage: python3 world_to_map.py <world.sdf> <output_basename>')
        print('  e.g. python3 scripts/world_to_map.py worlds/lab_world.sdf ~/map')
        sys.exit(1)

    sdf_path = sys.argv[1]
    out_base = os.path.expanduser(sys.argv[2])
    pgm_path = out_base + '.pgm'
    yaml_path = out_base + '.yaml'

    walls = parse_walls(sdf_path)
    if not walls:
        print(f'No <model name="wall_*"> boxes found in {sdf_path}')
        sys.exit(1)

    grid, (origin_x, origin_y) = rasterise(walls)
    H, W = grid.shape

    Image.fromarray(grid, mode='L').save(pgm_path)

    with open(yaml_path, 'w') as f:
        f.write(
            f'image: {os.path.basename(pgm_path)}\n'
            f'resolution: {RES}\n'
            f'origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.196\n'
        )

    print(f'Walls rasterised: {len(walls)}')
    print(f'Grid: {W}x{H} px @ {RES} m/px')
    print(f'Origin (world min corner): ({origin_x:.3f}, {origin_y:.3f})')
    print(f'Saved: {pgm_path}')
    print(f'Saved: {yaml_path}')


if __name__ == '__main__':
    main()
