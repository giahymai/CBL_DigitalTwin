#!/usr/bin/env python3
"""
navigator.py — Autonomous Farm Zone Navigator with Obstacle Avoidance
======================================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10
"""

import math
import time
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

WAYPOINTS = [
    {'name': 'spray_zone_A',     'x':  0.0, 'y':  2.0, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'fertilize_zone_B', 'x':  3.0, 'y':  2.0, 'action': 'fertilize', 'pause_s': 2.0},
    {'name': 'spray_zone_C',     'x':  3.0, 'y': -1.0, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'fertilize_zone_D', 'x':  0.0, 'y': -1.0, 'action': 'fertilize', 'pause_s': 2.0},
]


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator_node')

        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('max_linear',        0.15)
        self.declare_parameter('max_angular',       0.5)
        self.declare_parameter('goal_tolerance',    0.25)
        self.declare_parameter('obstacle_distance', 0.45)
        self.declare_parameter('front_angle_deg',   40.0)

        self._max_lin   = float(self.get_parameter('max_linear').value)
        self._max_ang   = float(self.get_parameter('max_angular').value)
        self._goal_tol  = float(self.get_parameter('goal_tolerance').value)
        self._obs_dist  = float(self.get_parameter('obstacle_distance').value)
        self._front_deg = float(self.get_parameter('front_angle_deg').value)

        self._x = self._y = self._yaw = 0.0
        self._odom_ok     = False
        self._scan        = None
        self._navigating  = False
        self._current     = None
        self._completed   = []
        self._state       = 'idle'  # idle | driving | avoiding | backing | stuck

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._scan_cb, scan_qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)

        self._cmd_pub    = self.create_publisher(TwistStamped, '/cmd_vel_raw',      10)
        self._status_pub = self.create_publisher(String,       '/navigator/status', 10)

        self.create_service(Trigger, '/start_navigation', self._start_srv)
        self.create_service(Trigger, '/stop_navigation',  self._stop_srv)
        self.create_service(Trigger, '/nav_status',       self._status_srv)
        self.create_timer(2.0, self._broadcast)

        self.get_logger().info('Navigator Node started')
        for wp in WAYPOINTS:
            self.get_logger().info(
                f'  [{wp["action"].upper():<11}] {wp["name"]} at ({wp["x"]}, {wp["y"]})'
            )

    def _scan_cb(self, msg): self._scan = msg
    def _odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._x, self._y = p.x, p.y
        self._yaw  = math.atan2(
            2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self._odom_ok = True

    def _start_srv(self, req, res):
        if self._navigating:
            res.success = False; res.message = 'Already navigating'; return res
        if not self._odom_ok:
            res.success = False; res.message = 'No odometry yet'; return res
        self._navigating = True
        self._completed  = []
        threading.Thread(target=self._run_all, daemon=True).start()
        res.success = True; res.message = f'Started — {len(WAYPOINTS)} zones'; return res

    def _stop_srv(self, req, res):
        self._navigating = False; self._current = None; self._state = 'idle'
        self._publish(0.0, 0.0)
        res.success = True; res.message = 'Stopped'; return res

    def _status_srv(self, req, res):
        remaining = [w['name'] for w in WAYPOINTS if w['name'] not in self._completed]
        res.success = True
        res.message = (f'state={self._state}\ncurrent={self._current}\n'
                       f'completed={self._completed}\nremaining={remaining}\n'
                       f'pos=({self._x:.2f},{self._y:.2f})')
        return res

    def _broadcast(self):
        msg = String()
        msg.data = (f'state={self._state} | current={self._current} | '
                    f'completed={self._completed}')
        self._status_pub.publish(msg)

    # ── Main navigation loop ───────────────────────────────────────────────

    def _run_all(self):
        for wp in WAYPOINTS:
            if not self._navigating:
                break
            self._current = wp['name']
            self.get_logger().info(f'[NAV] → {wp["name"]} at ({wp["x"]}, {wp["y"]})')
            if self._drive_to(wp['x'], wp['y']):
                self._publish(0.0, 0.0)
                self.get_logger().info(f'[ARRIVED] {wp["name"]} — pausing {wp["pause_s"]}s')
                time.sleep(wp['pause_s'])
                self._completed.append(wp['name'])
            else:
                self.get_logger().warn(f'[SKIP] {wp["name"]}')

        self._publish(0.0, 0.0)
        self._navigating = False
        self._current    = None
        self._state      = 'idle'
        self.get_logger().info(
            f'[DONE] {len(self._completed)}/{len(WAYPOINTS)} zones visited'
        )

    def _drive_to(self, tx: float, ty: float, timeout=90.0) -> bool:
        t_start     = time.monotonic()
        last_pos    = (self._x, self._y)
        t_progress  = time.monotonic()

        while self._navigating:
            elapsed = time.monotonic() - t_start
            if elapsed > timeout:
                self.get_logger().warn('Timeout')
                return False

            dist = math.hypot(tx - self._x, ty - self._y)
            if dist < self._goal_tol:
                return True

            # Stuck detection: if not moved 0.1m in 4 seconds → escape
            moved = math.hypot(self._x - last_pos[0], self._y - last_pos[1])
            if time.monotonic() - t_progress > 4.0:
                if moved < 0.08:
                    self.get_logger().warn('[STUCK] Not moving — executing escape')
                    self._escape()
                last_pos   = (self._x, self._y)
                t_progress = time.monotonic()

            obs, turn_dir = self._check_obstacle()

            if obs:
                self._state = 'avoiding'
                # Step 1: back up slightly
                self._publish(-self._max_lin * 0.6, 0.0)
                time.sleep(0.4)
                # Step 2: turn away from obstacle
                self._publish(0.0, turn_dir * self._max_ang)
                time.sleep(0.6)
            else:
                self._state = 'driving'
                bearing     = math.atan2(ty - self._y, tx - self._x)
                heading_err = self._wrap(bearing - self._yaw)
                angular     = max(-self._max_ang, min(self._max_ang, 1.5 * heading_err))
                linear      = self._max_lin if abs(heading_err) < 0.5 else 0.0
                self._publish(linear, angular)

            time.sleep(0.1)

        return False

    def _escape(self):
        """Escape routine when stuck: back up + turn a larger angle."""
        self._state = 'stuck'
        self.get_logger().info('[ESCAPE] Backing up + turning')

        # Back up
        self._publish(-self._max_lin, 0.0)
        time.sleep(0.8)

        # Check which side has more space and turn that way
        _, turn_dir = self._check_obstacle()

        # Turn significantly
        self._publish(0.0, turn_dir * self._max_ang)
        time.sleep(1.2)

        self._publish(0.0, 0.0)

    def _check_obstacle(self) -> Tuple[bool, float]:
        if self._scan is None:
            return False, 1.0

        msg  = self._scan
        rad  = math.radians(self._front_deg)
        left, right = [], []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min:
                continue
            a = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(a) <= rad:
                (left if a > 0 else right).append(r)

        front = left + right
        if not front or min(front) >= self._obs_dist:
            return False, 1.0

        mean_l = sum(left)  / len(left)  if left  else 0.0
        mean_r = sum(right) / len(right) if right else 0.0
        turn   = 1.0 if mean_l >= mean_r else -1.0

        self.get_logger().info(
            f'[OBS] min={min(front):.2f}m | '
            f'L={mean_l:.2f} R={mean_r:.2f} | '
            f'turn={"left" if turn>0 else "right"}',
            throttle_duration_sec=1.0,
        )
        return True, turn

    def _publish(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = linear
        msg.twist.angular.z = angular
        self._cmd_pub.publish(msg)

    @staticmethod
    def _wrap(a): return math.atan2(math.sin(a), math.cos(a))


def main():
    rclpy.init()
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()