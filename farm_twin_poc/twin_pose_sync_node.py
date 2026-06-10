#!/usr/bin/env python3
"""
twin_pose_sync_node.py  —  Node: Twin Pose Sync (pin the Gazebo twin to the
real robot so it never drifts)
============================================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

WHY
---
In the LAB Nav2 stack the Gazebo twin runs OPEN-LOOP: it only receives the
mirrored /sim/cmd_vel and integrates it with its own physics. The real robot
slips, lags over WiFi, and is continuously corrected by AMCL, so the two
diverge over a tour. Once the twin drifts, its /sim/scan "sees" walls where the
real robot has none -> twin_safety_node reads sim_blocked=True and cuts the real
robot's forward motion (while still allowing rotation) -> the robot lurches
left/right in place ("ngao").

This node kills the drift at the source. Every tick it reads the real robot's
localized pose (TF map->base_link, published by AMCL) and teleports the Gazebo
twin model to that exact pose. The twin becomes a faithful shadow of the real
robot, so /sim/scan reflects the real robot's true surroundings and the
bidirectional safety (Goal 2) stays meaningful WITHOUT phantom stops.

Only needed in the lab Nav2 stack (navigation.launch.py), where a real robot +
AMCL exist. At home the robot IS the Gazebo model, so there is nothing to pin.

HOW
---
Teleport uses the Gazebo Transport service `/world/<world>/set_pose`
(gz.msgs.Pose -> gz.msgs.Boolean), invoked through the `gz service` CLI. This is
self-contained (no extra ros_gz service bridge needed) and matches the
documented gz teleport method. The blocking CLI call runs on a worker thread so
it never stalls the TF/ROS callbacks.

NOTE: the gz MODEL name is independent of ROS namespaces. If teleport does
nothing, list the models with `gz model --list` and pass the right name via the
`entity_name` parameter (launch arg `twin_entity`).
"""
import math
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TwinPoseSyncNode(Node):

    def __init__(self):
        super().__init__('twin_pose_sync_node')

        # World + model names as known to Gazebo (NOT the ROS namespace).
        self.declare_parameter('world_name',   'lab_world')
        self.declare_parameter('entity_name',  'turtlebot3_burger')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame',  'base_link')
        self.declare_parameter('rate_hz',      10.0)
        self.declare_parameter('z',            0.01)   # keep the model on the floor

        self._world   = self.get_parameter('world_name').value
        self._entity  = self.get_parameter('entity_name').value
        self._gframe  = self.get_parameter('global_frame').value
        self._rframe  = self.get_parameter('robot_frame').value
        self._rate    = float(self.get_parameter('rate_hz').value)
        self._z       = float(self.get_parameter('z').value)
        self._service = f'/world/{self._world}/set_pose'

        self._gz = shutil.which('gz')
        if self._gz is None:
            self.get_logger().error(
                "'gz' CLI not found on PATH — cannot teleport the twin. "
                "Run this node where Gazebo is installed.")

        self._pose = None                 # latest (x, y, yaw) in the map frame
        self._lock = threading.Lock()
        self._stop = False

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # The ROS timer only reads TF (cheap, non-blocking). The blocking
        # `gz service` call runs on a worker thread so it never stalls callbacks.
        self.create_timer(1.0 / max(1.0, self._rate), self._read_pose)
        self._worker = threading.Thread(target=self._sync_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'Twin Pose Sync started | pinning gz model "{self._entity}" in world '
            f'"{self._world}" to {self._gframe}->{self._rframe} at {self._rate:.0f} Hz')

    def _read_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._gframe, self._rframe, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return  # AMCL/TF not ready yet — try again next tick
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose = (t.transform.translation.x,
                          t.transform.translation.y,
                          yaw)

    def _sync_loop(self):
        period = 1.0 / max(1.0, self._rate)
        while not self._stop and self._gz is not None:
            with self._lock:
                pose = self._pose
            if pose is not None:
                self._teleport(*pose)
            time.sleep(period)

    def _teleport(self, x, y, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        req = (f'name: "{self._entity}", '
               f'position: {{x: {x:.4f}, y: {y:.4f}, z: {self._z:.4f}}}, '
               f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}')
        try:
            subprocess.run(
                [self._gz, 'service', '-s', self._service,
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '200',
                 '--req', req],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=1.0)
        except Exception as e:                       # noqa: BLE001 — best-effort teleport
            self.get_logger().warn(f'set_pose failed: {e}', throttle_duration_sec=5.0)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main():
    rclpy.init()
    node = TwinPoseSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
