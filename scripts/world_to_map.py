#!/usr/bin/env python3
"""
world_to_map.py — bake a Nav2 occupancy grid from a Gazebo .world / .sdf
========================================================================
Farm Twin PoC | Team 5 Terra Minds

Reads an SDF/world file, rasterises every wall <box> visual (skipping the
ground plane and zone discs) into an occupancy grid, and writes the
standard Nav2 map pair:

  <name>.pgm   — P5 binary, 255 = free, 0 = occupied
  <name>.yaml  — image / resolution / origin / thresholds

Stdlib only. Shares parsing helpers with scripts/world_to_canvas.py.

USAGE
-----
    python3 scripts/world_to_map.py worlds/new_world.world
        -> writes maps/new_world_map.pgm and maps/new_world_map.yaml

    python3 scripts/world_to_map.py worlds/new_world.world \\
        --out maps/foo.pgm --resolution 0.05 --inflate 0.05

ARGS
----
  --out         output PGM path (default: maps/new_world_map.pgm).
                YAML is written alongside with the same stem.
  --resolution  metres per pixel (default: 0.05 — Nav2 standard).
  --padding     extra free space around the wall AABB, metres
                (default: 0.5).
  --inflate     thicken every wall by this many metres on all sides
                (default: 0.05). Walls in new_world are only 5 cm
                thick; without inflation a wall can fall between pixel
                centres and disappear. 5 cm = 1 px at default res, so
                walls always span ≥1 px. Costmap inflation is separate.

NAV2 NOTES
----------
Origin is the world position of the lower-left corner of the map. AMCL
matches incoming /scan against this grid, so the spawn pose in
gazebo_nav2_demo.launch.py + nav2_sim.yaml must lie inside the
rendered free space.

Zones (cylinders) are intentionally skipped — they're floor markers,
not obstacles, and AMCL shouldn't see them.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Optional


# ---------------------------------------------------------------------------
# SDF parsing (same conventions as scripts/world_to_canvas.py)
# ---------------------------------------------------------------------------

def _parse_pose(text: Optional[str]) -> tuple[float, float, float, float]:
    if not text:
        return (0.0, 0.0, 0.0, 0.0)
    parts = (text.strip().split() + ['0'] * 6)[:6]
    return (float(parts[0]), float(parts[1]),
            float(parts[2]), float(parts[5]))


def _compose(parent, child):
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy,
            py + s * cx + c * cy,
            pz + cz,
            pyaw + cyaw)


def _local_pose(elem: ET.Element):
    pose = elem.find('pose')
    return _parse_pose(pose.text if pose is not None else None)


_SKIP_MODELS = {'ground_plane', 'sun'}


def _extract_walls(world_path: str):
    """Return (x, y, yaw, w, h) for every <box> visual in the world."""
    tree = ET.parse(world_path)
    root = tree.getroot()
    worlds = root.findall('.//world') or [root]
    walls: list[tuple[float, float, float, float, float]] = []
    for w in worlds:
        for model in w.findall('model'):
            if model.get('name', '') in _SKIP_MODELS:
                continue
            mp = _local_pose(model)
            for link in model.findall('link'):
                lp = _compose(mp, _local_pose(link))
                for vis in link.findall('visual'):
                    vp = _compose(lp, _local_pose(vis))
                    geom = vis.find('geometry')
                    if geom is None:
                        continue
                    box = geom.find('box')
                    if box is None:
                        continue
                    size = box.find('size')
                    if size is None or not size.text:
                        continue
                    sx, sy, *_ = [float(v) for v in size.text.split()]
                    walls.append((vp[0], vp[1], vp[3], sx, sy))
    return walls


# ---------------------------------------------------------------------------
# rasterisation
# ---------------------------------------------------------------------------

def _aabb(walls, padding):
    xs: list[float] = []
    ys: list[float] = []
    for x, y, yaw, w, h in walls:
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        ew = w * c + h * s
        eh = w * s + h * c
        xs += [x - ew / 2, x + ew / 2]
        ys += [y - eh / 2, y + eh / 2]
    return (min(xs) - padding, min(ys) - padding,
            max(xs) + padding, max(ys) + padding)


def _rasterise(walls, resolution: float, padding: float, inflate: float):
    if not walls:
        raise SystemExit('no wall geometry found in world')

    min_x, min_y, max_x, max_y = _aabb(walls, padding)
    width  = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))
    # Snap world bounds up to an integer pixel count; origin stays put.
    max_x = min_x + width  * resolution
    max_y = min_y + height * resolution

    FREE, OCC = 255, 0
    grid = bytearray(bytes([FREE]) * (width * height))

    for x, y, yaw, w, h in walls:
        # Inflate so thin walls (~resolution) reliably cover ≥1 pixel even
        # when they happen to sit between pixel centres.
        we, he = w + inflate, h + inflate
        hw2, hh2 = we / 2.0, he / 2.0
        c, s = math.cos(yaw), math.sin(yaw)
        ac, as_ = abs(c), abs(s)
        # AABB of the rotated, inflated rectangle in world frame.
        eaw = we * ac + he * as_
        eah = we * as_ + he * ac
        bx0, bx1 = x - eaw / 2, x + eaw / 2
        by0, by1 = y - eah / 2, y + eah / 2
        # PGM row 0 = top of image = max_y, row height-1 = bottom = min_y.
        pj0 = max(0,          int(math.floor((bx0 - min_x) / resolution)))
        pj1 = min(width  - 1, int(math.ceil ((bx1 - min_x) / resolution)))
        pi0 = max(0,          int(math.floor((max_y - by1) / resolution)))
        pi1 = min(height - 1, int(math.ceil ((max_y - by0) / resolution)))

        for pi in range(pi0, pi1 + 1):
            wy = max_y - (pi + 0.5) * resolution
            row_off = pi * width
            for pj in range(pj0, pj1 + 1):
                wx = min_x + (pj + 0.5) * resolution
                dx, dy = wx - x, wy - y
                # Transform into the box's local frame (rotate by -yaw).
                lx =  c * dx + s * dy
                ly = -s * dx + c * dy
                if abs(lx) <= hw2 and abs(ly) <= hh2:
                    grid[row_off + pj] = OCC

    return {
        'data':       bytes(grid),
        'width':      width,
        'height':     height,
        'origin_x':   min_x,
        'origin_y':   min_y,
        'resolution': resolution,
    }


# ---------------------------------------------------------------------------
# PGM / YAML writers
# ---------------------------------------------------------------------------

def _write_pgm(path: str, width: int, height: int, data: bytes) -> None:
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{width} {height}\n'.encode('ascii'))
        f.write(b'255\n')
        f.write(data)


def _write_yaml(path: str, pgm_name: str, resolution: float,
                origin: tuple[float, float]) -> None:
    with open(path, 'w') as f:
        f.write(f'image: {pgm_name}\n')
        f.write(f'resolution: {resolution}\n')
        f.write(f'origin: [{origin[0]}, {origin[1]}, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: Optional[list[str]] = None) -> int:
    here = os.path.abspath(os.path.dirname(__file__))
    repo = os.path.dirname(here)
    default_out = os.path.join(repo, 'maps', 'new_world_map.pgm')

    p = argparse.ArgumentParser(
        description='Rasterise a Gazebo world into a Nav2 occupancy grid.')
    p.add_argument(
        'world',
        help='path to the .world or .sdf file (e.g. worlds/new_world.world)')
    p.add_argument(
        '--out', default=default_out,
        help=f'output PGM path (default: {os.path.relpath(default_out)})')
    p.add_argument(
        '--resolution', type=float, default=0.05,
        help='metres per pixel (default: 0.05)')
    p.add_argument(
        '--padding', type=float, default=0.5,
        help='extra free space around the wall AABB, metres (default: 0.5)')
    p.add_argument(
        '--inflate', type=float, default=0.05,
        help='thicken each wall by this many metres before rasterising '
             '(default: 0.05 = 1 px at default resolution)')
    args = p.parse_args(argv)

    if not os.path.isfile(args.world):
        print(f'error: world not found: {args.world}', file=sys.stderr)
        return 2

    walls = _extract_walls(args.world)
    grid = _rasterise(walls, args.resolution, args.padding, args.inflate)

    pgm_path  = args.out
    yaml_path = os.path.splitext(pgm_path)[0] + '.yaml'
    os.makedirs(os.path.dirname(pgm_path), exist_ok=True)
    _write_pgm(pgm_path,
               grid['width'], grid['height'], grid['data'])
    _write_yaml(yaml_path,
                os.path.basename(pgm_path),
                grid['resolution'],
                (grid['origin_x'], grid['origin_y']))

    occ = grid['data'].count(0)
    print(
        f'walls: {len(walls)}  '
        f'grid: {grid["width"]}x{grid["height"]} px  '
        f'({grid["width"] * grid["height"]} cells, {occ} occupied)\n'
        f'origin: ({grid["origin_x"]:.3f}, {grid["origin_y"]:.3f}) m  '
        f'resolution: {grid["resolution"]} m/px\n'
        f'wrote {os.path.relpath(pgm_path)}\n'
        f'wrote {os.path.relpath(yaml_path)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
