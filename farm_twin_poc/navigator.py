#!/usr/bin/env python3
"""
navigator.py — Autonomous Farm Zone Navigator
=============================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Features:
  - Autonomous navigation to farm zones in sequence
  - Reactive obstacle avoidance with backup + escape
  - Battery monitoring → auto return home when low
  - /return_home service → manual return to start position
  - Improved heading control (pre-rotate before driving)
"""

import math
import time
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

WAYPOINTS = [
    {'name': 'spray_zone_A',     'x':  0.5, 'y':  2.7, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'fertilize_zone_B', 'x':  3.5, 'y':  2.7, 'action': 'fertilize', 'pause_s': 2.0},
    {'name': 'spray_zone_C',     'x':  3.5, 'y': 0.7, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'fertilize_zone_D', 'x':  0.5, 'y': 0.7, 'action': 'fertilize', 'pause_s': 2.0},
]

# Battery threshold — return home below this percentage
LOW_BATTERY_PERCENT = 20.0   # %
LOW_BATTERY_VOLTAGE = 11.0   # V (fallback if percentage not available)


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator_node')

        # Parameters
        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('max_linear',        0.15)
        self.declare_parameter('max_angular',       0.6)
        self.declare_parameter('goal_tolerance',    0.25)
        self.declare_parameter('obstacle_distance', 0.40)
        self.declare_parameter('front_angle_deg',   45.0)
        self.declare_parameter('battery_topic',     '/battery_state')

        self._max_lin   = float(self.get_parameter('max_linear').value)
        self._max_ang   = float(self.get_parameter('max_angular').value)
        self._goal_tol  = float(self.get_parameter('goal_tolerance').value)
        self._obs_dist  = float(self.get_parameter('obstacle_distance').value)
        self._front_deg = float(self.get_parameter('front_angle_deg').value)

        # Robot state
        self._x = self._y = self._yaw = 0.0
        self._home_x = self._home_y = None   # recorded at first odom message
        self._odom_ok   = False
        self._scan      = None
        self._battery_pct = 100.0
        self._battery_v   = 12.0
        self._low_battery = False

        # Navigation state
        self._navigating  = False
        self._current     = None
        self._completed   = []
        self._state       = 'idle'
        self._returning   = False

        # QoS for LiDAR
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._scan_cb, scan_qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(
            BatteryState, self.get_parameter('battery_topic').value,
            self._battery_cb, 10)

        # Publishers
        self._cmd_pub    = self.create_publisher(TwistStamped, '/cmd_vel_raw',      10)
        self._status_pub = self.create_publisher(String,       '/navigator/status', 10)

        # Services
        self.create_service(Trigger, '/start_navigation', self._start_srv)
        self.create_service(Trigger, '/stop_navigation',  self._stop_srv)
        self.create_service(Trigger, '/return_home',      self._return_home_srv)
        self.create_service(Trigger, '/nav_status',       self._status_srv)

        self.create_timer(2.0, self._broadcast)

        self.get_logger().info('Navigator Node started')
        self.get_logger().info(
            f'  scan={self.get_parameter("scan_topic").value} | '
            f'odom={self.get_parameter("odom_topic").value}'
        )
        self.get_logger().info(f'  {len(WAYPOINTS)} waypoints loaded')
        self.get_logger().info(
            f'  Battery threshold: {LOW_BATTERY_PERCENT}% / {LOW_BATTERY_VOLTAGE}V'
        )
        self.get_logger().info(
            '  Services: /start_navigation | /stop_navigation | /return_home | /nav_status'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _scan_cb(self, msg): self._scan = msg

    def _odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._x, self._y = p.x, p.y
        self._yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        if not self._odom_ok:
            # Record home position on first odom message
            self._home_x = p.x
            self._home_y = p.y
            self.get_logger().info(
                f'Home position recorded: ({self._home_x:.3f}, {self._home_y:.3f})'
            )
        self._odom_ok = True

    def _battery_cb(self, msg: BatteryState):
        self._battery_pct = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
        self._battery_v   = msg.voltage

        low = (self._battery_pct < LOW_BATTERY_PERCENT or
               self._battery_v   < LOW_BATTERY_VOLTAGE)

        if low and not self._low_battery and not self._returning:
            self._low_battery = True
            self.get_logger().warn(
                f'[BATTERY LOW] {self._battery_pct:.0f}% / {self._battery_v:.1f}V '
                f'→ Auto return home!'
            )
            threading.Thread(target=self._auto_return_home, daemon=True).start()

    # ── Services ───────────────────────────────────────────────────────────

    def _start_srv(self, req, res):
        if self._navigating or self._returning:
            res.success = False
            res.message = 'Already navigating. Call /stop_navigation first.'
            return res
        if not self._odom_ok:
            res.success = False
            res.message = 'No odometry yet.'
            return res
        self._navigating = True
        self._completed  = []
        threading.Thread(target=self._run_all, daemon=True).start()
        res.success = True
        res.message = f'Navigation started — {len(WAYPOINTS)} zones queued'
        return res

    def _stop_srv(self, req, res):
        self._navigating = False
        self._returning  = False
        self._current    = None
        self._state      = 'idle'
        self._publish(0.0, 0.0)
        res.success = True
        res.message = 'Navigation stopped'
        return res

    def _return_home_srv(self, req, res):
        if self._returning:
            res.success = False
            res.message = 'Already returning home'
            return res
        if self._home_x is None:
            res.success = False
            res.message = 'Home position not recorded yet'
            return res
        # Cancel current navigation
        self._navigating = False
        time.sleep(0.3)
        threading.Thread(target=self._go_home, daemon=True).start()
        res.success = True
        res.message = f'Returning home to ({self._home_x:.2f}, {self._home_y:.2f})'
        self.get_logger().info(res.message)
        return res

    def _status_srv(self, req, res):
        remaining = [w['name'] for w in WAYPOINTS if w['name'] not in self._completed]
        res.success = True
        res.message = (
            f'state={self._state}\n'
            f'current={self._current}\n'
            f'returning_home={self._returning}\n'
            f'completed={self._completed}\n'
            f'remaining={remaining}\n'
            f'position=({self._x:.2f}, {self._y:.2f})\n'
            f'home=({self._home_x}, {self._home_y})\n'
            f'battery={self._battery_pct:.0f}% / {self._battery_v:.1f}V'
        )
        return res

    def _broadcast(self):
        msg = String()
        msg.data = (
            f'state={self._state} | '
            f'current={self._current} | '
            f'returning={self._returning} | '
            f'battery={self._battery_pct:.0f}% | '
            f'completed={self._completed}'
        )
        self._status_pub.publish(msg)

    # ── Navigation logic ───────────────────────────────────────────────────

    def _run_all(self):
        self.get_logger().info(f'[NAV START] {len(WAYPOINTS)} zones')
        for wp in WAYPOINTS:
            if not self._navigating:
                break
            self._current = wp['name']
            self.get_logger().info(
                f'[NAV] → {wp["name"]} at ({wp["x"]}, {wp["y"]})'
            )
            if self._drive_to(wp['x'], wp['y']):
                self._publish(0.0, 0.0)
                self.get_logger().info(
                    f'[ARRIVED] {wp["name"]} — {wp["action"].upper()} '
                    f'pausing {wp["pause_s"]}s'
                )
                time.sleep(wp['pause_s'])
                self._completed.append(wp['name'])
                self.get_logger().info(
                    f'[DONE] {wp["name"]} | '
                    f'{len(self._completed)}/{len(WAYPOINTS)}'
                )
            else:
                self.get_logger().warn(f'[SKIP] {wp["name"]}')

        self._publish(0.0, 0.0)
        self._navigating = False
        self._current    = None
        self._state      = 'idle'
        self.get_logger().info(
            f'[NAV COMPLETE] {len(self._completed)}/{len(WAYPOINTS)} zones visited'
        )

    def _auto_return_home(self):
        """Triggered automatically when battery is low."""
        self._navigating = False
        time.sleep(0.5)
        self._go_home()

    def _go_home(self):
        """Navigate back to home position (0,0 or recorded start)."""
        if self._home_x is None:
            return
        self._returning = True
        self._state     = 'returning_home'
        self.get_logger().info(
            f'[RETURN HOME] Navigating to ({self._home_x:.2f}, {self._home_y:.2f})'
        )
        success = self._drive_to(self._home_x, self._home_y, timeout=120.0)
        self._publish(0.0, 0.0)
        if success:
            self.get_logger().info('[HOME] Robot returned to start position')
        else:
            self.get_logger().warn('[HOME] Could not reach start position')
        self._returning = False
        self._state     = 'idle'

    def _drive_to(self, tx: float, ty: float, timeout=90.0) -> bool:
        """
        Improved navigation:
          1. Pre-rotate if heading error > 90° (avoids driving sideways)
          2. Drive with proportional control
          3. Obstacle avoidance: backup + turn
          4. Stuck detection + escape
        """
        t_start    = time.monotonic()
        last_pos   = (self._x, self._y)
        t_progress = time.monotonic()

        while self._navigating or self._returning:
            if time.monotonic() - t_start > timeout:
                self.get_logger().warn('Timeout')
                return False

            dist = math.hypot(tx - self._x, ty - self._y)
            if dist < self._goal_tol:
                return True

            # Stuck detection
            moved = math.hypot(self._x - last_pos[0], self._y - last_pos[1])
            if time.monotonic() - t_progress > 4.0:
                if moved < 0.08:
                    self.get_logger().warn('[STUCK] Executing escape')
                    self._escape()
                last_pos   = (self._x, self._y)
                t_progress = time.monotonic()

            obs, turn_dir = self._check_obstacle()

            if obs:
                # Obstacle: back up then turn
                self._state = 'avoiding'
                self._publish(-self._max_lin * 0.6, 0.0)
                time.sleep(0.4)
                self._publish(0.0, turn_dir * self._max_ang)
                time.sleep(0.5)
            else:
                bearing     = math.atan2(ty - self._y, tx - self._x)
                heading_err = self._wrap(bearing - self._yaw)

                if abs(heading_err) > 1.2:
                    # Large heading error → rotate in place first
                    self._state = 'rotating'
                    self._publish(0.0, math.copysign(self._max_ang, heading_err))
                else:
                    # Drive with proportional heading correction
                    self._state   = 'driving'
                    angular = max(-self._max_ang,
                                  min(self._max_ang, 1.8 * heading_err))
                    # Scale speed by heading error — smoother curves
                    scale   = max(0.3, 1.0 - abs(heading_err) / math.pi)
                    linear  = self._max_lin * scale
                    self._publish(linear, angular)

            time.sleep(0.1)

        return False

    def _escape(self):
        """Back up + large turn to escape stuck situation."""
        self._state = 'escaping'
        self._publish(-self._max_lin, 0.0)
        time.sleep(1.0)
        _, turn_dir = self._check_obstacle()
        self._publish(0.0, turn_dir * self._max_ang)
        time.sleep(1.5)
        self._publish(0.0, 0.0)

    def _check_obstacle(self) -> Tuple[bool, float]:
        """
        Check front arc for obstacles.
        Returns (obstacle_detected, turn_direction)
        turn_direction: +1 = left, -1 = right
        """
        if self._scan is None:
            return False, 1.0

        msg = self._scan
        rad = math.radians(self._front_deg)

        left_min = float('inf')
        right_min = float('inf')
        front_min = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            a = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(a) > rad:
                continue
            front_min = min(front_min, r)
            if a > 0:
                left_min = min(left_min, r)
            else:
                right_min = min(right_min, r)

        if front_min >= self._obs_dist:
            return False, 1.0

        # Turn toward side with more clearance
        turn_dir = 1.0 if left_min >= right_min else -1.0

        self.get_logger().info(
            f'[OBS] front={front_min:.2f}m | '
            f'L={left_min:.2f} R={right_min:.2f} | '
            f'turn={"left" if turn_dir > 0 else "right"}',
            throttle_duration_sec=1.0,
        )
        return True, turn_dir

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