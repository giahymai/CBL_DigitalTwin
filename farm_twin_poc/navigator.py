#!/usr/bin/env python3
"""
navigator.py — Autonomous Farm Zone Navigator with Obstacle Avoidance
======================================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Navigates to each farm zone in sequence using odometry + LiDAR.
Does NOT require Nav2 or a pre-built map.

Behaviour:
  1. Drive toward current waypoint using proportional heading control
  2. If obstacle detected within obstacle_distance → turn away from it
  3. On arrival at zone → pause 2s (zone_monitor detects and logs action)
  4. Move to next zone until all visited

Commands published to /cmd_vel_raw → processed by twin_safety_node
→ forwarded to /cmd_vel (real robot) AND /sim/cmd_vel (Gazebo twin)
→ State Synchronisation maintained during autonomous navigation

Topics:
  SUB  scan_topic   — LiDAR data (default: /scan, use /sim/scan in Gazebo)
  SUB  odom_topic   — robot position (default: /odom, use /sim/odom in Gazebo)
  PUB  /cmd_vel_raw — velocity commands → twin_safety_node

Services:
  /start_navigation — start autonomous run through all farm zones
  /stop_navigation  — cancel navigation
  /nav_status       — query current progress

WAYPOINTS must match FARM_ZONES in zone_monitor_node.py (same x, y).
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

# ── WAYPOINTS ──────────────────────────────────────────────────────────────
# Must match FARM_ZONES in zone_monitor_node.py
WAYPOINTS = [
    {'name': 'spray_zone_A',     'x':  3.0, 'y':  3.0, 'action': 'spray',      'pause_s': 2.0},
    {'name': 'fertilize_zone_B', 'x':  0.0, 'y':  1.0, 'action': 'fertilize',  'pause_s': 2.0},
    {'name': 'spray_zone_C',     'x': -1.0, 'y':  0.0, 'action': 'spray',      'pause_s': 2.0},
    {'name': 'fertilize_zone_D', 'x':  0.0, 'y': -1.0, 'action': 'fertilize',  'pause_s': 2.0},
]


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator_node')

        # Parameters
        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('max_linear',        0.15)   # m/s forward speed
        self.declare_parameter('max_angular',       0.5)    # rad/s turn speed
        self.declare_parameter('goal_tolerance',    0.25)   # metres — arrival radius
        self.declare_parameter('obstacle_distance', 0.45)   # metres — avoidance trigger
        self.declare_parameter('front_angle_deg',   40.0)   # degrees — front arc

        self._max_linear    = float(self.get_parameter('max_linear').value)
        self._max_angular   = float(self.get_parameter('max_angular').value)
        self._goal_tol      = float(self.get_parameter('goal_tolerance').value)
        self._obs_dist      = float(self.get_parameter('obstacle_distance').value)
        self._front_deg     = float(self.get_parameter('front_angle_deg').value)

        # State
        self._x:           float = 0.0
        self._y:           float = 0.0
        self._yaw:         float = 0.0
        self._odom_ok:     bool  = False
        self._latest_scan: Optional[LaserScan] = None
        self._navigating:  bool  = False
        self._current:     Optional[str] = None
        self._completed:   list  = []
        self._avoiding:    bool  = False

        # Subscribers
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._scan_cb, scan_qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_cb, 10)

        # Publishers
        self._cmd_pub    = self.create_publisher(TwistStamped, '/cmd_vel_raw',       10)
        self._status_pub = self.create_publisher(String,       '/navigator/status',  10)

        # Services
        self.create_service(Trigger, '/start_navigation', self._start_srv)
        self.create_service(Trigger, '/stop_navigation',  self._stop_srv)
        self.create_service(Trigger, '/nav_status',       self._status_srv)

        # Status broadcast timer
        self.create_timer(2.0, self._broadcast)

        self.get_logger().info('Navigator Node started (no Nav2 required)')
        self.get_logger().info(f'  scan={self.get_parameter("scan_topic").value}')
        self.get_logger().info(f'  odom={self.get_parameter("odom_topic").value}')
        self.get_logger().info(f'  {len(WAYPOINTS)} waypoints loaded:')
        for wp in WAYPOINTS:
            self.get_logger().info(
                f'    [{wp["action"].upper():<11}] {wp["name"]} '
                f'at ({wp["x"]}, {wp["y"]}) pause={wp["pause_s"]}s'
            )
        self.get_logger().info(
            'Start: ros2 service call /start_navigation std_srvs/srv/Trigger'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._x   = p.x
        self._y   = p.y
        self._yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._odom_ok = True

    # ── Services ───────────────────────────────────────────────────────────

    def _start_srv(self, req, res):
        if self._navigating:
            res.success = False
            res.message = 'Already navigating. Call /stop_navigation first.'
            return res
        if not self._odom_ok:
            res.success = False
            res.message = 'No odometry received yet. Check odom_topic parameter.'
            return res
        self._navigating = True
        self._completed  = []
        threading.Thread(target=self._run_all, daemon=True).start()
        res.success = True
        res.message = f'Navigation started — {len(WAYPOINTS)} zones queued'
        return res

    def _stop_srv(self, req, res):
        self._navigating = False
        self._current    = None
        self._avoiding   = False
        self._publish_cmd(0.0, 0.0)
        res.success = True
        res.message = 'Navigation stopped'
        return res

    def _status_srv(self, req, res):
        remaining = [w['name'] for w in WAYPOINTS if w['name'] not in self._completed]
        res.success = True
        res.message = (
            f'navigating={self._navigating}\n'
            f'current={self._current}\n'
            f'avoiding={self._avoiding}\n'
            f'completed={self._completed}\n'
            f'remaining={remaining}\n'
            f'position=({self._x:.3f}, {self._y:.3f})'
        )
        return res

    def _broadcast(self) -> None:
        remaining = [w['name'] for w in WAYPOINTS if w['name'] not in self._completed]
        msg      = String()
        msg.data = (
            f'navigating={self._navigating} | '
            f'current={self._current} | '
            f'avoiding={self._avoiding} | '
            f'completed={self._completed} | '
            f'remaining={remaining}'
        )
        self._status_pub.publish(msg)

    # ── Navigation logic ───────────────────────────────────────────────────

    def _run_all(self) -> None:
        self.get_logger().info(
            f'[NAV START] Autonomous run through {len(WAYPOINTS)} farm zones'
        )
        for wp in WAYPOINTS:
            if not self._navigating:
                break
            self._current = wp['name']
            self.get_logger().info(
                f'[NAV → {wp["name"]}] Driving to ({wp["x"]}, {wp["y"]})'
            )
            success = self._drive_to(wp['x'], wp['y'])
            if not self._navigating:
                break
            if success:
                # Arrived — pause to simulate spray/fertilize
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    f'[ARRIVED] {wp["name"]} | '
                    f'{wp["action"].upper()} — pausing {wp["pause_s"]}s'
                )
                time.sleep(wp['pause_s'])
                self._completed.append(wp['name'])
                self.get_logger().info(
                    f'[DONE] {wp["name"]} | '
                    f'{len(self._completed)}/{len(WAYPOINTS)} complete'
                )
            else:
                self.get_logger().warn(
                    f'[SKIP] Could not reach {wp["name"]}'
                )

        self._publish_cmd(0.0, 0.0)
        self._navigating = False
        self._current    = None
        self._avoiding   = False
        self.get_logger().info(
            f'[NAV COMPLETE] Visited {len(self._completed)}/{len(WAYPOINTS)} zones: '
            f'{self._completed}'
        )

    def _drive_to(self, tx: float, ty: float, timeout: float = 60.0) -> bool:
        """Drive to (tx, ty) with reactive obstacle avoidance. Returns True on arrival."""
        t_start = time.monotonic()

        while self._navigating:
            if time.monotonic() - t_start > timeout:
                self.get_logger().warn('Timeout reaching waypoint')
                return False

            dist = math.sqrt((tx - self._x)**2 + (ty - self._y)**2)

            # Arrived?
            if dist < self._goal_tol:
                return True

            # Check for obstacle
            obs_ahead, turn_dir = self._check_obstacle()

            if obs_ahead:
                # Obstacle avoidance: turn away
                self._avoiding = True
                self._publish_cmd(0.0, turn_dir * self._max_angular)
            else:
                self._avoiding = False
                # Proportional heading control toward goal
                bearing     = math.atan2(ty - self._y, tx - self._x)
                heading_err = self._wrap(bearing - self._yaw)
                angular     = max(-self._max_angular,
                                  min(self._max_angular, 1.5 * heading_err))
                # Only drive forward when roughly facing goal
                linear = self._max_linear if abs(heading_err) < 0.4 else 0.0
                self._publish_cmd(linear, angular)

            time.sleep(0.1)  # 10 Hz control loop

        return False

    def _check_obstacle(self) -> Tuple[bool, float]:
        """
        Returns (obstacle_ahead, turn_direction).
        turn_direction: +1 = turn left, -1 = turn right
        """
        if self._latest_scan is None:
            return False, 1.0

        msg  = self._latest_scan
        rad  = math.radians(self._front_deg)
        left_ranges  = []
        right_ranges = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min:
                continue
            a = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(a) <= rad:
                if a > 0:
                    left_ranges.append(r)
                else:
                    right_ranges.append(r)

        all_front = left_ranges + right_ranges
        if not all_front:
            return False, 1.0

        min_front = min(all_front)
        if min_front >= self._obs_dist:
            return False, 1.0

        # Turn toward the side with more clearance
        mean_left  = sum(left_ranges)  / len(left_ranges)  if left_ranges  else 0.0
        mean_right = sum(right_ranges) / len(right_ranges) if right_ranges else 0.0
        turn_dir   = 1.0 if mean_left >= mean_right else -1.0

        self.get_logger().info(
            f'[OBSTACLE] min={min_front:.2f}m | '
            f'left={mean_left:.2f} right={mean_right:.2f} | '
            f'turning {"left" if turn_dir > 0 else "right"}',
            throttle_duration_sec=1.0,
        )
        return True, turn_dir

    # ── Helpers ────────────────────────────────────────────────────────────

    def _publish_cmd(self, linear: float, angular: float) -> None:
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = linear
        msg.twist.angular.z = angular
        self._cmd_pub.publish(msg)

    @staticmethod
    def _wrap(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


def main():
    rclpy.init()
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()