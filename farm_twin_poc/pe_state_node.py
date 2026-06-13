#!/usr/bin/env python3
"""
pe_state_node.py  —  Physical Entity core (Option B)
====================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

In Option B there is NO physical robot: the Gazebo TurtleBot3 IS the "physical
entity" (PE) — the source of truth. But Gazebo only gives us /scan and /odom; a
real robot also exposes INTERNAL state (battery, sensor health, motor status,
mode) and can SUFFER FAULTS. This node simulates that physical layer so the
Digital Entity (the web dashboard) has real state to synchronise with, and so a
fault injected from the dashboard actually changes the robot's behaviour.

It does three jobs:

1. INTERNAL STATE (Goal ② state-sync, not just commands)
   Maintains and publishes /pe/state (JSON): battery %, lidar_health,
   motor_status, mode, localization_quality, position. Also publishes a real
   sensor_msgs/BatteryState on /battery_state so the navigator's low-battery
   return-home actually fires.

2. COMMAND GATE (Goal ① DE→PE control + safety)
   It is the ONLY publisher of /cmd_vel (the command Gazebo executes). Every
   tick it resolves the command from, in priority order:
     a. EMERGENCY STOP  — zero, if lidar FAILED / motor STALL / battery empty
     b. DASHBOARD TELEOP — /de/cmd_vel, if a fresh teleop command arrived
     c. NAV2            — /cmd_vel_pe (Nav2 collision_monitor output)
   So the dashboard can drive the robot, and any fault immediately stops it.

3. SENSOR FAULT INJECTION (Goal ② fault-injection + Goal ③ sensing change)
   Subscribes raw /scan from Gazebo and republishes /scan_filtered — the scan
   the costmaps, collision_monitor and dashboard actually use:
     OK       — pass through unchanged
     DEGRADED — drop ~half the beams + add noise (Nav2 copes, viz looks sparse)
     FAILED   — empty scan (robot is emergency-stopped by the gate)

Fault injection comes in on /de/command (JSON), e.g.
  {"target": "lidar",   "action": "fail"}      -> lidar_health = FAILED
  {"target": "lidar",   "action": "degrade"}   -> lidar_health = DEGRADED
  {"target": "lidar",   "action": "clear"}     -> lidar_health = OK
  {"target": "motor",   "action": "stall"}     -> motor_status = STALL (stop)
  {"target": "motor",   "action": "clear"}
  {"target": "battery", "action": "set", "value": 15}   -> battery % = 15
  {"target": "battery", "action": "drain"}     -> fast drain
  {"target": "system",  "action": "reset"}     -> clear every fault
"""
import json
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class PEStateNode(Node):

    def __init__(self):
        super().__init__('pe_state_node')

        # ---- topics ----
        self.declare_parameter('nav_cmd_topic',      '/cmd_vel_pe')   # from Nav2
        self.declare_parameter('teleop_cmd_topic',   '/de/cmd_vel')   # from dashboard
        self.declare_parameter('out_cmd_topic',      '/cmd_vel')      # to Gazebo
        self.declare_parameter('raw_scan_topic',     '/scan')         # from Gazebo
        self.declare_parameter('filtered_scan_topic','/scan_filtered')# to system
        self.declare_parameter('odom_topic',         '/odom')
        self.declare_parameter('nav_status_topic',   '/navigator/status')
        self.declare_parameter('state_topic',        '/pe/state')
        self.declare_parameter('battery_topic',      '/battery_state')
        self.declare_parameter('command_topic',      '/de/command')

        # ---- behaviour tuning ----
        self.declare_parameter('cmd_rate_hz',        20.0)
        self.declare_parameter('state_rate_hz',      4.0)
        self.declare_parameter('teleop_timeout_s',   0.6)
        self.declare_parameter('battery_start_pct',  100.0)
        self.declare_parameter('drain_idle_pct_s',   0.05)  # %/s standing still
        self.declare_parameter('drain_move_pct_s',   0.30)  # %/s while moving
        self.declare_parameter('drain_fault_pct_s',  3.0)   # %/s when fault-drained
        self.declare_parameter('degrade_drop_frac',  0.5)   # beams dropped (DEGRADED)
        self.declare_parameter('degrade_noise_m',    0.08)  # range noise (DEGRADED)

        gp = self.get_parameter
        self._teleop_timeout = float(gp('teleop_timeout_s').value)
        self._battery_pct    = float(gp('battery_start_pct').value)
        self._drain_idle     = float(gp('drain_idle_pct_s').value)
        self._drain_move     = float(gp('drain_move_pct_s').value)
        self._drain_fault    = float(gp('drain_fault_pct_s').value)
        self._degrade_drop   = float(gp('degrade_drop_frac').value)
        self._degrade_noise  = float(gp('degrade_noise_m').value)

        # ---- fault / health state ----
        self._lidar_health = 'OK'        # OK | DEGRADED | FAILED
        self._motor_status = 'OK'        # OK | STALL
        self._battery_drain_fault = False
        self._nav_mode     = 'idle'      # mirrored from /navigator/status
        self._nearest_obstacle = float('inf')

        # ---- cached commands ----
        self._nav_cmd    = None          # latest Twist from Nav2
        self._nav_t      = 0.0           # time of last Nav2 command
        self._teleop_cmd = None          # latest Twist from dashboard
        self._teleop_t   = 0.0           # time of last teleop command
        self._cmd_timeout = 0.5          # s; ignore a command source once it goes stale
        self._moving     = False
        self._x = self._y = self._yaw = 0.0
        self._emergency  = False

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- subscriptions ----
        self.create_subscription(TwistStamped, gp('nav_cmd_topic').value,
                                 self._nav_cmd_cb, 10)
        self.create_subscription(TwistStamped, gp('teleop_cmd_topic').value,
                                 self._teleop_cb, 10)
        self.create_subscription(LaserScan, gp('raw_scan_topic').value,
                                 self._scan_cb, scan_qos)
        self.create_subscription(Odometry, gp('odom_topic').value,
                                 self._odom_cb, 10)
        self.create_subscription(String, gp('nav_status_topic').value,
                                 self._nav_status_cb, 10)
        self.create_subscription(String, gp('command_topic').value,
                                 self._command_cb, 10)

        # ---- publishers ----
        self._cmd_pub   = self.create_publisher(TwistStamped, gp('out_cmd_topic').value, 10)
        self._scan_pub  = self.create_publisher(LaserScan, gp('filtered_scan_topic').value, scan_qos)
        self._state_pub = self.create_publisher(String, gp('state_topic').value, 10)
        self._batt_pub  = self.create_publisher(BatteryState, gp('battery_topic').value, 10)

        # ---- timers ----
        self.create_timer(1.0 / max(1.0, float(gp('cmd_rate_hz').value)),   self._cmd_tick)
        self.create_timer(1.0 / max(1.0, float(gp('state_rate_hz').value)), self._state_tick)
        self.create_timer(1.0, self._battery_tick)   # drain once per second

        self.get_logger().info('PE State Node started (Physical Entity core)')
        self.get_logger().info(
            f'  cmd gate: {gp("nav_cmd_topic").value} / {gp("teleop_cmd_topic").value} '
            f'-> {gp("out_cmd_topic").value}')
        self.get_logger().info(
            f'  scan fault: {gp("raw_scan_topic").value} -> {gp("filtered_scan_topic").value}')
        self.get_logger().info(
            f'  state: {gp("state_topic").value} | battery: {gp("battery_topic").value} '
            f'| faults in: {gp("command_topic").value}')

    # ------------------------------------------------------------------ inputs
    def _nav_cmd_cb(self, msg: TwistStamped):
        self._nav_cmd = msg.twist
        self._nav_t = self._now()

    def _teleop_cb(self, msg: TwistStamped):
        self._teleop_cmd = msg.twist
        self._teleop_t = self._now()

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _nav_status_cb(self, msg: String):
        # message looks like: "state=navigating | current=... | battery=..."
        for tok in msg.data.split('|'):
            tok = tok.strip()
            if tok.startswith('state='):
                self._nav_mode = tok.split('=', 1)[1].strip()
                break

    def _command_cb(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            self.get_logger().warn(f'bad /de/command (not JSON): {msg.data!r}')
            return
        target = str(cmd.get('target', '')).lower()
        action = str(cmd.get('action', '')).lower()

        if target == 'lidar':
            if action in ('fail', 'failed'):       self._lidar_health = 'FAILED'
            elif action in ('degrade', 'degraded'):self._lidar_health = 'DEGRADED'
            elif action in ('clear', 'ok', 'fix'): self._lidar_health = 'OK'
        elif target == 'motor':
            if action in ('stall', 'fail'):        self._motor_status = 'STALL'
            elif action in ('clear', 'ok', 'fix'): self._motor_status = 'OK'
        elif target == 'battery':
            if action == 'set':
                try:    self._battery_pct = max(0.0, min(100.0, float(cmd.get('value', 100))))
                except (TypeError, ValueError): pass
            elif action == 'drain':                self._battery_drain_fault = True
            elif action in ('clear', 'ok', 'fix'): self._battery_drain_fault = False
        elif target == 'system' and action == 'reset':
            self._lidar_health = 'OK'
            self._motor_status = 'OK'
            self._battery_drain_fault = False

        self.get_logger().info(
            f'[FAULT] {target}:{action} -> lidar={self._lidar_health} '
            f'motor={self._motor_status} batt={self._battery_pct:.0f}%')

    # ------------------------------------------------------------- command gate
    def _cmd_tick(self):
        self._emergency = (self._lidar_health == 'FAILED'
                           or self._motor_status == 'STALL'
                           or self._battery_pct <= 0.0)

        now = self._now()
        twist = None
        if self._emergency:
            twist = None  # -> zero
        elif (now - self._teleop_t) < self._teleop_timeout and self._teleop_cmd:
            twist = self._teleop_cmd
        elif self._nav_cmd is not None and (now - self._nav_t) < self._cmd_timeout:
            twist = self._nav_cmd       # only while Nav2 is actively commanding

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        if twist is not None:
            out.twist = twist
        self._cmd_pub.publish(out)

        self._moving = (twist is not None
                        and (abs(out.twist.linear.x) > 0.01 or abs(out.twist.angular.z) > 0.01))

    # --------------------------------------------------------------- scan fault
    def _scan_cb(self, msg: LaserScan):
        out = msg                      # default: pass through (OK)
        if self._lidar_health != 'OK':
            out = LaserScan()
            out.header = msg.header
            out.angle_min = msg.angle_min
            out.angle_max = msg.angle_max
            out.angle_increment = msg.angle_increment
            out.time_increment = msg.time_increment
            out.scan_time = msg.scan_time
            out.range_min = msg.range_min
            out.range_max = msg.range_max
            out.intensities = list(msg.intensities)
            if self._lidar_health == 'FAILED':
                out.ranges = [float('inf')] * len(msg.ranges)
                out.intensities = []
            else:  # DEGRADED
                ranges = []
                for r in msg.ranges:
                    if random.random() < self._degrade_drop:
                        ranges.append(float('inf'))            # dropped beam
                    elif math.isfinite(r):
                        ranges.append(r + random.gauss(0.0, self._degrade_noise))
                    else:
                        ranges.append(r)
                out.ranges = ranges
        self._scan_pub.publish(out)

        # metric: nearest obstacle in the (healthy) scan, for the dashboard
        valid = [r for r in msg.ranges
                 if math.isfinite(r) and msg.range_min < r < msg.range_max]
        self._nearest_obstacle = min(valid) if valid else float('inf')

    # ------------------------------------------------------------------ battery
    def _battery_tick(self):
        if self._battery_pct <= 0.0:
            self._battery_pct = 0.0
            return
        if self._battery_drain_fault:
            rate = self._drain_fault
        else:
            rate = self._drain_move if self._moving else self._drain_idle
        self._battery_pct = max(0.0, self._battery_pct - rate)

    # -------------------------------------------------------------------- state
    def _mode(self):
        if self._emergency:
            return 'SAFE_STOP'
        if (self._now() - self._teleop_t) < self._teleop_timeout:
            return 'teleop'
        return self._nav_mode or 'idle'

    def _loc_quality(self):
        return {'OK': 'GOOD', 'DEGRADED': 'DEGRADED', 'FAILED': 'LOST'}[self._lidar_health]

    def _state_tick(self):
        state = {
            'battery_pct':          round(self._battery_pct, 1),
            'lidar_health':         self._lidar_health,
            'motor_status':         self._motor_status,
            'mode':                 self._mode(),
            'localization_quality': self._loc_quality(),
            'emergency_stop':       self._emergency,
            'nearest_obstacle_m':   (round(self._nearest_obstacle, 3)
                                     if math.isfinite(self._nearest_obstacle) else None),
            'position':             {'x': round(self._x, 3), 'y': round(self._y, 3),
                                     'yaw': round(self._yaw, 3)},
        }
        self._state_pub.publish(String(data=json.dumps(state)))

        batt = BatteryState()
        batt.header.stamp = self.get_clock().now().to_msg()
        batt.percentage = self._battery_pct / 100.0      # 0..1, as on a real TB3
        batt.present = True
        self._batt_pub.publish(batt)

    def _now(self):
        # monotonic-ish seconds from the ROS clock (wall time on a real PE)
        t = self.get_clock().now().nanoseconds
        return t * 1e-9


def main():
    rclpy.init()
    node = PEStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
