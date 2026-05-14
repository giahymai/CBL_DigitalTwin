#!/usr/bin/env python3
"""
twin_safety_node.py  —  Node 2: Digital Twin Safety
====================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Based on course tb3_scanner_dt/twin_safety_node.py.
Extended with /get_twin_status Service.

ROS concepts:
  Node        — Digital Entity process
  Topic (SUB) — /scan        (real robot LiDAR)
  Topic (SUB) — /sim/scan    (Gazebo twin LiDAR via PushRosNamespace)
  Topic (SUB) — /cmd_vel_raw (teleop input)
  Topic (PUB) — /cmd_vel     (safe command → real robot)   ← Bi-directional
  Topic (PUB) — /sim/cmd_vel (same command → Gazebo twin)  ← State Sync
  Service     — /get_twin_status (query DE state on demand)

DTAS criteria:
  Bi-directional Communication   — /scan PE→DE, /cmd_vel DE→PE
  State Synchronisation          — identical command to /cmd_vel AND /sim/cmd_vel
  Object/Environment Interaction — stop if obstacle < stop_distance
"""
import json
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger


class TwinSafetyNode(Node):

    def __init__(self):
        super().__init__('twin_safety_node')

        self.declare_parameter('real_scan_topic',  '/scan')
        self.declare_parameter('sim_scan_topic',   '/sim/scan')
        self.declare_parameter('input_cmd_topic',  '/cmd_vel_raw')
        self.declare_parameter('real_cmd_topic',   '/cmd_vel')
        self.declare_parameter('sim_cmd_topic',    '/sim/cmd_vel')
        self.declare_parameter('stop_distance',    0.25)
        self.declare_parameter('front_angle_deg',  30.0)

        self.stop_distance   = float(self.get_parameter('stop_distance').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)

        self.real_blocked      = False
        self.sim_blocked       = False
        self.real_min_distance = float('inf')
        self.sim_min_distance  = float('inf')
        self._cmd_count        = 0
        self._block_count      = 0

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(LaserScan,    self.get_parameter('real_scan_topic').value,  self.real_scan_cb, scan_qos)
        self.create_subscription(LaserScan,    self.get_parameter('sim_scan_topic').value,   self.sim_scan_cb,  scan_qos)
        self.create_subscription(TwistStamped, self.get_parameter('input_cmd_topic').value,  self.cmd_cb,       10)

        self.real_pub = self.create_publisher(TwistStamped, self.get_parameter('real_cmd_topic').value, 10)
        self.sim_pub  = self.create_publisher(TwistStamped, self.get_parameter('sim_cmd_topic').value,  10)

        self.create_service(Trigger, '/get_twin_status', self._status_srv)

        self.get_logger().info('Twin Safety Node started')
        self.get_logger().info(
            f'SUB: {self.get_parameter("real_scan_topic").value} | '
            f'{self.get_parameter("sim_scan_topic").value} | '
            f'{self.get_parameter("input_cmd_topic").value}'
        )
        self.get_logger().info(
            f'PUB: {self.get_parameter("real_cmd_topic").value} | '
            f'{self.get_parameter("sim_cmd_topic").value} → State Synchronisation'
        )
        self.get_logger().info(
            'SRV: /get_twin_status — ros2 service call /get_twin_status std_srvs/srv/Trigger'
        )

    def real_scan_cb(self, msg):
        self.real_min_distance, self.real_blocked = self._evaluate(msg)

    def sim_scan_cb(self, msg):
        self.sim_min_distance, self.sim_blocked = self._evaluate(msg)

    def _evaluate(self, msg):
        front = self._front_arc(msg)
        valid = [r for r in front if math.isfinite(r) and msg.range_min < r < msg.range_max]
        if not valid:
            return float('inf'), False
        d = min(valid)
        return d, d < self.stop_distance

    def _front_arc(self, msg: LaserScan) -> List[float]:
        rad = math.radians(self.front_angle_deg)
        result = []
        for i, r in enumerate(msg.ranges):
            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= rad:
                result.append(r)
        return result

    def cmd_cb(self, msg):
        blocked = (self.real_blocked or self.sim_blocked) and msg.twist.linear.x > 0.0

        safe = TwistStamped()
        safe.header = msg.header
        if blocked:
            safe.twist.angular.z = msg.twist.angular.z  # allow turning
            self.get_logger().warn(
                f'STOP | real_min={self.real_min_distance:.2f} '
                f'sim_min={self.sim_min_distance:.2f} '
                f'threshold={self.stop_distance:.2f}',
                throttle_duration_sec=1.0,
            )
            self._block_count += 1
        else:
            safe = msg

        self.get_logger().info(
            f'real_blocked={self.real_blocked} sim_blocked={self.sim_blocked} '
            f'real_min={self.real_min_distance:.2f} sim_min={self.sim_min_distance:.2f} '
            f'lin.x={msg.twist.linear.x:.2f}',
            throttle_duration_sec=1.0,
        )

        # STATE SYNCHRONISATION: same command to both worlds simultaneously
        self.real_pub.publish(safe)
        self.sim_pub.publish(safe)
        self._cmd_count += 1

    def _status_srv(self, request, response):
        response.success = True
        response.message = json.dumps({
            'real_blocked':        self.real_blocked,
            'sim_blocked':         self.sim_blocked,
            'real_min_m':          round(self.real_min_distance, 3) if math.isfinite(self.real_min_distance) else 'inf',
            'sim_min_m':           round(self.sim_min_distance, 3)  if math.isfinite(self.sim_min_distance)  else 'inf',
            'stop_threshold_m':    self.stop_distance,
            'commands_processed':  self._cmd_count,
            'times_blocked':       self._block_count,
        }, indent=2)
        return response


def main():
    rclpy.init()
    node = TwinSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
