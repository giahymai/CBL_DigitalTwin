#!/usr/bin/env python3
"""
navigator.py
 
A ROS 2 navigator node built around a GOAL STACK (LIFO).
 
Behaviour
---------
1. Goals are received on a topic (default '/nav_goal', geometry_msgs/Point)
   and pushed onto a stack. The robot always drives toward the TOP of the
   stack and pops it when reached.
 
2. While driving, the front LiDAR sector is monitored. When an obstacle is
   detected within `obstacle_distance`, the obstacle-avoidance routine
   computes a "safe point" (a detour waypoint to one side of the obstacle)
   and PUSHES it onto the stack. Because the stack is LIFO, the robot goes
   to the safe point FIRST, then automatically resumes its previous goal(s).
 
The avoidance routine is an improved version of the front-sector line-fit
idea: it fits a line to the obstacle surface in front (to estimate the
wall's orientation) and steers toward whichever side has more clearance.
 
NOTE: This is a working framework, not a tuned controller. The gains and
distances below are conservative starting points — expect to tune them.
"""
 
import math
import numpy as np
 
import rclpy
from rclpy.node import Node
 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
 
 
class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
 
        # ---------------- Parameters ----------------
        # Topic wiring (overridable at launch)
        self.declare_parameter('goal_topic',      '/nav_goal')
        self.declare_parameter('real_scan_topic', '/scan')
        self.declare_parameter('sim_scan_topic',  '/sim/scan')
        self.declare_parameter('use_sim_scan',    False)
        self.declare_parameter('odom_topic',      '/odom')
        self.declare_parameter('cmd_vel_topic',   '/cmd_vel')
 
        # Behaviour tuning
        self.declare_parameter('obstacle_distance', 0.50)  # trigger range [m]
        self.declare_parameter('front_half_angle',  0.52)  # +/- ~30 deg [rad]
        self.declare_parameter('detour_distance',   0.70)  # safe-pt offset [m]
        self.declare_parameter('escape_angle',      1.20)  # ~70 deg [rad]
        self.declare_parameter('goal_tolerance',    0.15)  # arrival radius [m]
        self.declare_parameter('avoid_cooldown',    3.0)   # s between detours
        self.declare_parameter('max_linear',        0.15)  # m/s
        self.declare_parameter('max_angular',       0.60)  # rad/s
        self.declare_parameter('heading_tol',       0.30)  # rad; turn-in-place
 
        gp = self.get_parameter
        self.obstacle_distance = gp('obstacle_distance').value
        self.front_half_angle  = gp('front_half_angle').value
        self.detour_distance   = gp('detour_distance').value
        self.escape_angle      = gp('escape_angle').value
        self.goal_tolerance    = gp('goal_tolerance').value
        self.avoid_cooldown    = gp('avoid_cooldown').value
        self.max_linear        = gp('max_linear').value
        self.max_angular       = gp('max_angular').value
        self.heading_tol       = gp('heading_tol').value
 
        scan_topic = (gp('sim_scan_topic').value if gp('use_sim_scan').value
                      else gp('real_scan_topic').value)
 
        # ---------------- State ----------------
        # Stack entries are dicts: {'x', 'y', 'safe'(bool)}
        self.goal_stack = []
        self.pose = None              # (x, y, yaw) once odom arrives
        self.handling_obstacle = False
        self.last_avoid_time = 0.0
 
        # ---------------- I/O ----------------
        self.goal_sub = self.create_subscription(
            Point, gp('goal_topic').value, self.goal_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, gp('odom_topic').value, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, gp('cmd_vel_topic').value, 10)
 
        # Control loop @ 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
 
        self.latest_scan = None
        self.get_logger().info(
            f"Navigator up. goals='{gp('goal_topic').value}', "
            f"scan='{scan_topic}', odom='{gp('odom_topic').value}'")
 
    # ============================================================
    # Subscriptions
    # ============================================================
    def goal_callback(self, msg: Point):
        """Push every received goal onto the stack."""
        self.goal_stack.append({'x': msg.x, 'y': msg.y, 'safe': False})
        self.get_logger().info(
            f"Goal pushed ({msg.x:.2f}, {msg.y:.2f}); "
            f"stack depth = {len(self.goal_stack)}")
 
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.pose = (p.x, p.y, yaw)
 
    def scan_callback(self, msg: LaserScan):
        # Just cache it; detection happens in the control loop so that
        # obstacle handling stays synchronised with driving decisions.
        self.latest_scan = msg
 
    # ============================================================
    # Control loop
    # ============================================================
    def control_loop(self):
        # Need a pose to do anything meaningful.
        if self.pose is None:
            return
 
        # Nothing to do -> hold still.
        if not self.goal_stack:
            self.stop()
            return
 
        # --- Obstacle check (only if not already detouring) ---
        if (self.latest_scan is not None
                and not self.handling_obstacle
                and self._cooldown_elapsed()):
            if self.front_min_distance(self.latest_scan) < self.obstacle_distance:
                safe = self.compute_safe_point(self.latest_scan)
                if safe is not None:
                    self.goal_stack.append(
                        {'x': safe[0], 'y': safe[1], 'safe': True})
                    self.handling_obstacle = True
                    self.last_avoid_time = self._now()
                    self.get_logger().info(
                        f"Obstacle! detour -> ({safe[0]:.2f}, {safe[1]:.2f})")
 
        # --- Drive toward top of stack ---
        target = self.goal_stack[-1]
        px, py, yaw = self.pose
        dx, dy = target['x'] - px, target['y'] - py
        dist = math.hypot(dx, dy)
 
        if dist < self.goal_tolerance:
            reached = self.goal_stack.pop()
            if reached['safe']:
                self.handling_obstacle = False  # detour done, resume goals
            self.get_logger().info(
                f"Reached {'safe point' if reached['safe'] else 'goal'}; "
                f"stack depth = {len(self.goal_stack)}")
            self.stop()
            return
 
        # Proportional steering: face the target, then advance.
        bearing = math.atan2(dy, dx)
        heading_err = self._wrap(bearing - yaw)
 
        cmd = Twist()
        cmd.angular.z = max(-self.max_angular,
                            min(self.max_angular, 1.5 * heading_err))
        # Only move forward when roughly facing the target.
        if abs(heading_err) < self.heading_tol:
            cmd.linear.x = min(self.max_linear, 0.5 * dist)
        self.cmd_pub.publish(cmd)
 
    # ============================================================
    # Obstacle-avoidance logic (improved from your front-fit idea)
    # ============================================================
    def front_min_distance(self, msg: LaserScan):
        """Smallest valid range within +/- front_half_angle of straight ahead."""
        best = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min:
                continue
            ang = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(ang) <= self.front_half_angle:
                best = min(best, r)
        return best
 
    def compute_safe_point(self, msg: LaserScan):
        """
        Return a detour waypoint (x, y) in the ODOM frame, or None.
 
        Strategy:
          * Build (angle, range) for the front hemisphere.
          * Fit a line to the obstacle points to estimate wall orientation
            (this is the improved version of your polyfit step).
          * Pick the side (left/right) with greater clearance.
          * Place the safe point `detour_distance` away along an escape
            bearing toward the open side.
        """
        angles, ranges, pts = [], [], []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min:
                continue
            ang = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(ang) <= math.pi / 2:                 # front hemisphere
                angles.append(ang)
                ranges.append(r)
                # Standard ROS frame: x forward, y left.
                pts.append((r * math.cos(ang), r * math.sin(ang)))
 
        if len(angles) < 4:
            return None
 
        # --- Fit a line to nearby (obstacle) points for wall orientation ---
        buffer = 0.30
        obst = [p for p, r in zip(pts, ranges)
                if r < self.obstacle_distance + buffer]
        wall_angle = None
        if len(obst) >= 2:
            xs = np.array([p[0] for p in obst])
            ys = np.array([p[1] for p in obst])
            # Fit y = m*x + b; wall direction angle = atan2(m, 1).
            try:
                m, _b = np.polyfit(xs, ys, 1)
                wall_angle = math.atan2(m, 1.0)
            except Exception:
                wall_angle = None
 
        # --- Which side is more open? Compare mean clearance L vs R ---
        left  = [r for a, r in zip(angles, ranges) if a > 0.0]
        right = [r for a, r in zip(angles, ranges) if a < 0.0]
        mean_left  = sum(left) / len(left) if left else 0.0
        mean_right = sum(right) / len(right) if right else 0.0
        side = +1.0 if mean_left >= mean_right else -1.0   # +1 left, -1 right
 
        # --- Escape bearing (robot frame) ---
        # Steer toward the open side; if we have a wall estimate, bias the
        # escape to run roughly parallel to the wall.
        escape = side * self.escape_angle
        if wall_angle is not None:
            escape = self._wrap(0.5 * escape + 0.5 * (wall_angle + side * math.pi / 2))
 
        rx = self.detour_distance * math.cos(escape)
        ry = self.detour_distance * math.sin(escape)
 
        # --- Transform robot-frame point -> odom frame ---
        px, py, yaw = self.pose
        ox = px + rx * math.cos(yaw) - ry * math.sin(yaw)
        oy = py + rx * math.sin(yaw) + ry * math.cos(yaw)
        return (ox, oy)
 
    # ============================================================
    # Helpers
    # ============================================================
    def stop(self):
        self.cmd_pub.publish(Twist())
 
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9
 
    def _cooldown_elapsed(self):
        return (self._now() - self.last_avoid_time) > self.avoid_cooldown
 
    @staticmethod
    def _wrap(a):
        """Normalise angle to [-pi, pi]."""
        return math.atan2(math.sin(a), math.cos(a))
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()