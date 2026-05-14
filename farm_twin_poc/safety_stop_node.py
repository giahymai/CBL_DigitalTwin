#!/usr/bin/env python3
"""
safety_stop_node.py  —  Node 1: LiDAR Safety Stop (simulation only)
====================================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Used for at-home simulation testing only.
For the real robot + Digital Twin, use twin_safety_node instead.

Topics:
  SUB /scan        — LiDAR data from Gazebo
  SUB /cmd_vel_raw — raw teleop input
  PUB /cmd_vel     — filtered safe command
"""
import math
from typing import List
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class SafetyStopNode(Node):

    def __init__(self):
        super().__init__('safety_stop_node')

        self.declare_parameter('scan_topic',       '/scan')
        self.declare_parameter('input_cmd_topic',  '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stop_distance',    0.25)
        self.declare_parameter('front_angle_deg',  30.0)

        self.stop_distance   = float(self.get_parameter('stop_distance').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.blocked         = False
        self.min_dist        = float('inf')

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan,    self.get_parameter('scan_topic').value,      self.scan_cb, scan_qos)
        self.create_subscription(TwistStamped, self.get_parameter('input_cmd_topic').value, self.cmd_cb,  10)
        self._pub = self.create_publisher(TwistStamped, self.get_parameter('output_cmd_topic').value, 10)

        self.get_logger().info(f'Safety Stop Node started | stop_distance={self.stop_distance} m')

    def scan_cb(self, msg):
        front = self.get_front_arc(msg, self.front_angle_deg)
        valid = [r for r in front if math.isfinite(r) and msg.range_min < r < msg.range_max]
        self.min_dist = min(valid) if valid else float('inf')
        self.blocked  = self.min_dist < self.stop_distance

    def get_front_arc(self, msg: LaserScan, deg: float) -> List[float]:
        rad = math.radians(deg)
        result = []
        for i, r in enumerate(msg.ranges):
            angle = math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                               math.cos(msg.angle_min + i * msg.angle_increment))
            if abs(angle) <= rad:
                result.append(r)
        return result

    def cmd_cb(self, msg):
        out = TwistStamped()
        out.header = msg.header
        if self.blocked and msg.twist.linear.x > 0.0:
            out.twist.angular.z = msg.twist.angular.z
            self.get_logger().info(
                f'[BLOCKED] min_front={self.min_dist:.2f} m', throttle_duration_sec=1.0)
        else:
            out = msg
        self._pub.publish(out)


def main():
    rclpy.init()
    node = SafetyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
