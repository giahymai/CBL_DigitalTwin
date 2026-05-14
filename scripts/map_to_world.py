#!/usr/bin/env python3
"""
map_to_world.py — SLAM Map → Gazebo SDF World
==============================================
Converts map.pgm + map.yaml (from SLAM) to a Gazebo SDF world file.
Includes required system plugins for TurtleBot3 LiDAR and movement.

Requirements: pip3 install pillow pyyaml numpy

Usage:
  cd ~/turtlebot3_ws/src/farm_twin_poc
  python3 scripts/map_to_world.py ~/map.pgm ~/map.yaml worlds/lab_world.sdf
  colcon build --packages-select farm_twin_poc && source install/setup.bash
"""
import sys, yaml, numpy as np
from PIL import Image

WALL_HEIGHT = 0.5
FREE_THRESH = 230
MIN_PIXELS  = 2

PLUGINS = """
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>"""


def load_map(pgm, yaml_path):
    with open(yaml_path) as f:
        info = yaml.safe_load(f)
    data = np.array(Image.open(pgm).convert('L'), dtype=np.uint8)
    return data, float(info['resolution']), info['origin']


def segments(data, res, origin):
    h, w = data.shape
    segs = []
    for row in range(h):
        in_wall = False
        for col in range(w + 1):
            occ = col < w and data[row, col] < FREE_THRESH
            if occ and not in_wall:
                start = col; in_wall = True
            elif not occ and in_wall:
                length = col - start
                if length >= MIN_PIXELS:
                    x = origin[0] + (start + length / 2.0) * res
                    y = origin[1] + (h - row - 0.5) * res
                    segs.append((x, y, length * res, res))
                in_wall = False
    return segs


def to_sdf(segs):
    models = ''.join(f"""
    <model name='wall_{i}'>
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {WALL_HEIGHT/2:.4f} 0 0 0</pose>
      <link name='link'>
        <collision name='col'><geometry><box><size>{w:.4f} {d:.4f} {WALL_HEIGHT:.4f}</size></box></geometry></collision>
        <visual name='vis'>
          <geometry><box><size>{w:.4f} {d:.4f} {WALL_HEIGHT:.4f}</size></box></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>""" for i, (x, y, w, d) in enumerate(segs))

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="lab_world">
{PLUGINS}

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="col"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
{models}
  </world>
</sdf>
"""


def main():
    if len(sys.argv) != 4:
        print('Usage: python3 map_to_world.py <map.pgm> <map.yaml> <output.sdf>')
        sys.exit(1)
    pgm, yaml_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
    data, res, origin = load_map(pgm, yaml_path)
    print(f'Map: {data.shape[1]}x{data.shape[0]} px @ {res} m/px')
    segs = segments(data, res, origin)
    print(f'Found {len(segs)} wall segments')
    with open(out, 'w') as f:
        f.write(to_sdf(segs))
    print(f'Saved: {out}')
    print('Next: colcon build --packages-select farm_twin_poc && source install/setup.bash')


if __name__ == '__main__':
    main()
