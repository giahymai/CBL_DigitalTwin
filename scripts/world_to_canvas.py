#!/usr/bin/env python3
"""
world_to_canvas.py — convert a Gazebo .world / .sdf to HTML-canvas data
======================================================================
Farm Twin PoC | Team 5 Terra Minds

Reads an SDF/world file and emits a JSON description of every visible
primitive (box / cylinder / sphere) projected onto the XY plane, so the
web UI can render the world on a <canvas> with no extra parsing logic.

Stdlib only — no rclpy, no lxml, no PIL.

USAGE
-----
    python3 scripts/world_to_canvas.py worlds/new_world.world
        -> writes web/map.json

    python3 scripts/world_to_canvas.py worlds/lab_world.sdf --out web/map.json
    python3 scripts/world_to_canvas.py worlds/new_world.world --stdout
    python3 scripts/world_to_canvas.py worlds/new_world.world --pretty

Re-run after editing the world; the web server picks up the new file on
the next page load (static cache is only 2 s).

OUTPUT (JSON)
-------------
{
  "source": "worlds/new_world.world",
  "bounds": {"min_x": ..., "min_y": ..., "max_x": ..., "max_y": ...},
  "shapes": [
    {"type":"box",    "name":"box_3", "x":..,"y":..,"w":..,"h":..,
     "yaw":.., "color":"rgba(r,g,b,a)"},
    {"type":"circle", "name":"spray_zone_A", "x":..,"y":..,"r":..,
     "color":"rgba(r,g,b,a)"},
    ...
  ]
}

All coordinates are in WORLD frame (metres). `yaw` is in radians, CCW.
`bounds` ignores the infinite ground plane.

RENDERING HINT
--------------
A minimal canvas pipeline in your app.js would be:
    const m = await (await fetch('/map.json')).json();
    const W = canvas.width, H = canvas.height;
    const dx = m.bounds.max_x - m.bounds.min_x;
    const dy = m.bounds.max_y - m.bounds.min_y;
    const px = Math.min(W/dx, H/dy);              // metres -> pixels
    const toX = x => (x - m.bounds.min_x) * px;
    const toY = y => H - (y - m.bounds.min_y) * px;  // y up in world
    // then iterate m.shapes and call ctx.fillRect / ctx.arc ...

This script does NOT pick a canvas size or compute pixels — it only gives
you world-space data, so the UI can pan/zoom freely.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Iterable, Optional


# ---------------------------------------------------------------------------
# pose composition (only the planar bits we need)
# ---------------------------------------------------------------------------

def _parse_pose(text: Optional[str]) -> tuple[float, float, float, float]:
    """Parse an SDF <pose> "x y z roll pitch yaw" into (x, y, z, yaw).

    Roll and pitch are dropped — we render a top-down map. The four-tuple
    matches what the rest of the script expects.
    """
    if not text:
        return (0.0, 0.0, 0.0, 0.0)
    parts = text.strip().split()
    parts = (parts + ['0'] * 6)[:6]
    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
    yaw = float(parts[5])
    return (x, y, z, yaw)


def _compose(parent: tuple[float, float, float, float],
             child:  tuple[float, float, float, float],
             ) -> tuple[float, float, float, float]:
    """world_pose(child) given parent's world pose and child's local pose.

    SDF nests model -> link -> visual; each <pose> is relative to its
    parent. We only need (x, y, z, yaw), so rotate the child's xy into the
    parent's yaw and add.
    """
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy,
            py + s * cx + c * cy,
            pz + cz,
            pyaw + cyaw)


# ---------------------------------------------------------------------------
# colour extraction
# ---------------------------------------------------------------------------

_DEFAULT_COLOR = (0.6, 0.6, 0.6, 1.0)


def _color_of(visual: ET.Element) -> tuple[float, float, float, float]:
    """Pull diffuse colour from a <visual>; fall back to ambient, then grey.

    SDF colour fields are space-separated "r g b a" with each in [0,1].
    """
    mat = visual.find('material')
    if mat is None:
        return _DEFAULT_COLOR
    for tag in ('diffuse', 'ambient'):
        node = mat.find(tag)
        if node is not None and node.text:
            try:
                vals = [float(v) for v in node.text.strip().split()]
                if len(vals) >= 3:
                    a = vals[3] if len(vals) >= 4 else 1.0
                    return (vals[0], vals[1], vals[2], a)
            except ValueError:
                pass
    return _DEFAULT_COLOR


def _rgba(c: tuple[float, float, float, float]) -> str:
    r, g, b, a = c
    return (f'rgba({int(round(r * 255))},'
            f'{int(round(g * 255))},'
            f'{int(round(b * 255))},'
            f'{a:.3f})')


# ---------------------------------------------------------------------------
# walk the SDF tree
# ---------------------------------------------------------------------------

# Models we never want on a 2D map (infinite ground, etc.).
_SKIP_MODELS = {'ground_plane', 'sun'}


def _local_pose(elem: ET.Element) -> tuple[float, float, float, float]:
    pose = elem.find('pose')
    return _parse_pose(pose.text if pose is not None else None)


def _iter_visuals(model: ET.Element,
                  model_pose: tuple[float, float, float, float]
                  ) -> Iterable[tuple[ET.Element,
                                      tuple[float, float, float, float],
                                      str]]:
    """Yield (visual_elem, world_pose, model_name) for each visual in model."""
    name = model.get('name', '')
    for link in model.findall('link'):
        link_pose = _compose(model_pose, _local_pose(link))
        for vis in link.findall('visual'):
            yield vis, _compose(link_pose, _local_pose(vis)), name


def _shape_from_visual(visual: ET.Element,
                       world_pose: tuple[float, float, float, float],
                       name: str,
                       ) -> Optional[dict]:
    """Convert a single <visual> into a shape dict, or None if unrenderable."""
    geom = visual.find('geometry')
    if geom is None:
        return None
    x, y, _z, yaw = world_pose
    color = _rgba(_color_of(visual))

    box = geom.find('box')
    if box is not None:
        size = box.find('size')
        if size is None or not size.text:
            return None
        sx, sy, *_ = [float(v) for v in size.text.strip().split()]
        return {
            'type':  'box',
            'name':  name,
            'x':     x,
            'y':     y,
            'w':     sx,
            'h':     sy,
            'yaw':   yaw,
            'color': color,
        }

    cyl = geom.find('cylinder')
    if cyl is not None:
        r = cyl.find('radius')
        if r is None or not r.text:
            return None
        return {
            'type':  'circle',
            'name':  name,
            'x':     x,
            'y':     y,
            'r':     float(r.text),
            'color': color,
        }

    sph = geom.find('sphere')
    if sph is not None:
        r = sph.find('radius')
        if r is None or not r.text:
            return None
        return {
            'type':  'circle',
            'name':  name,
            'x':     x,
            'y':     y,
            'r':     float(r.text),
            'color': color,
        }

    # plane/mesh/heightmap/etc. — not useful for a 2D top-down sketch.
    return None


def convert(world_path: str) -> dict:
    """Parse an SDF/world file and return the JSON-ready shape list."""
    tree = ET.parse(world_path)
    root = tree.getroot()
    # SDF root may be <sdf> wrapping one or more <world>. Accept either.
    worlds = root.findall('.//world') or [root]

    shapes: list[dict] = []
    for w in worlds:
        for model in w.findall('model'):
            if model.get('name', '') in _SKIP_MODELS:
                continue
            for vis, pose, name in _iter_visuals(model, _local_pose(model)):
                shape = _shape_from_visual(vis, pose, name)
                if shape is not None:
                    shapes.append(shape)

    bounds = _bounds(shapes)
    return {
        'source': os.path.relpath(world_path),
        'bounds': bounds,
        'shapes': shapes,
    }


def _bounds(shapes: Iterable[dict]) -> dict:
    """AABB over all shapes; ignores rotation (good enough for canvas fit)."""
    xs_min: list[float] = []
    ys_min: list[float] = []
    xs_max: list[float] = []
    ys_max: list[float] = []
    for s in shapes:
        if s['type'] == 'box':
            hw, hh = s['w'] / 2.0, s['h'] / 2.0
            xs_min.append(s['x'] - hw); xs_max.append(s['x'] + hw)
            ys_min.append(s['y'] - hh); ys_max.append(s['y'] + hh)
        else:  # circle
            r = s['r']
            xs_min.append(s['x'] - r); xs_max.append(s['x'] + r)
            ys_min.append(s['y'] - r); ys_max.append(s['y'] + r)
    if not xs_min:
        return {'min_x': 0.0, 'min_y': 0.0, 'max_x': 0.0, 'max_y': 0.0}
    return {
        'min_x': min(xs_min), 'min_y': min(ys_min),
        'max_x': max(xs_max), 'max_y': max(ys_max),
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: Optional[list[str]] = None) -> int:
    here = os.path.abspath(os.path.dirname(__file__))
    repo = os.path.dirname(here)
    default_out = os.path.join(repo, 'web', 'map.json')

    p = argparse.ArgumentParser(
        description='Convert a Gazebo .world/.sdf into JSON for HTML canvas.')
    p.add_argument(
        'world',
        help='path to the .world or .sdf file '
             '(e.g. worlds/new_world.world)')
    p.add_argument(
        '--out', default=default_out,
        help=f'output JSON path (default: {os.path.relpath(default_out)})')
    p.add_argument(
        '--stdout', action='store_true',
        help='print JSON to stdout instead of writing a file')
    p.add_argument(
        '--pretty', action='store_true',
        help='indent the JSON output (default: compact)')
    args = p.parse_args(argv)

    if not os.path.isfile(args.world):
        print(f'error: world not found: {args.world}', file=sys.stderr)
        return 2

    data = convert(args.world)
    indent = 2 if args.pretty else None
    text = json.dumps(data, indent=indent, allow_nan=False)

    if args.stdout:
        sys.stdout.write(text + '\n')
    else:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, 'w') as f:
            f.write(text + '\n')
        print(
            f'wrote {len(data["shapes"])} shapes -> '
            f'{os.path.relpath(args.out)}  '
            f'(bounds x:[{data["bounds"]["min_x"]:.2f},'
            f'{data["bounds"]["max_x"]:.2f}] '
            f'y:[{data["bounds"]["min_y"]:.2f},'
            f'{data["bounds"]["max_y"]:.2f}])')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
