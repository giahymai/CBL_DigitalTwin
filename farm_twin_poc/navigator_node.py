#!/usr/bin/env python3
"""
navigator_node.py  —  Node 5: Autonomous Navigator (Nav2 Simple Commander)
==========================================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Drives the TurtleBot3 autonomously to each farm zone IN SEQUENCE using the
Nav2 Simple Commander API (BasicNavigator). Nav2 plans the global path and
handles local obstacle avoidance, so the robot no longer "mo" (gropes) around
the room with hand-coded reactive logic.

Behaviour
---------
1. On /start_navigation: visit every WAYPOINT in order. Nav2 plans + follows
   the path. At each zone we pause briefly so zone_monitor_node fires the
   spray/fertilize /farm_action.
2. Auto return-home: while navigating, if battery < return_battery_percent
   (default 20 %) the current goal is CANCELLED and the robot drives back to
   the recorded home pose.
3. /return_home service: cancel whatever is running and go home, ANY time.
4. /stop_navigation: cancel and go idle (stays put).

Why waypoints match FARM_ZONES
-------------------------------
zone_monitor_node.py only emits /farm_action when the robot is physically
inside a zone radius. So WAYPOINTS here are aligned to FARM_ZONES; if you edit
one, edit the other to match.

Topics
  SUB  <odom_topic>          — current pose (default /odom)
  SUB  <battery_topic>       — sensor_msgs/BatteryState (default /battery_state)
  PUB  /navigator/status     — human-readable progress (every 3 s)

Services (std_srvs/Trigger)
  /start_navigation — visit all zones in sequence
  /return_home      — cancel + drive to home pose now
  /stop_navigation  — cancel + idle
  /nav_status       — query state

Run order
  1. ros2 launch farm_twin_poc gazebo_nav2_demo.launch.py map:=$HOME/map.yaml
       (or, real robot:  ros2 launch farm_twin_poc navigation.launch.py map:=~/map.yaml)
  2. Wait for "Nav2 is active". If set_initial_pose:=false, click
     "2D Pose Estimate" in RViz at the robot's real spot first.
  3. ros2 service call /start_navigation std_srvs/srv/Trigger
"""
import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Aligned with FARM_ZONES in zone_monitor_node.py. yaw is the heading the robot
# should face on arrival (radians); it does not affect zone detection.
WAYPOINTS = [
    {'name': 'spray_zone_A',     'x': 0.5, 'y': 2.7, 'yaw': 0.0, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'fertilize_zone_B', 'x': 3.5, 'y': 2.7, 'yaw': 0.0, 'action': 'fertilize', 'pause_s': 2.0},
    {'name': 'spray_zone_C',     'x': 3.5, 'y': 0.7, 'yaw': 0.0, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'fertilize_zone_D', 'x': 0.5, 'y': 0.7, 'yaw': 0.0, 'action': 'fertilize', 'pause_s': 2.0},
]


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator_node')

        # ---- parameters ----
        self.declare_parameter('odom_topic',             '/odom')
        self.declare_parameter('battery_topic',          '/battery_state')
        self.declare_parameter('return_battery_percent', 20.0)
        self.declare_parameter('battery_check_period',   2.0)
        self.declare_parameter('set_initial_pose',       False)
        # Home pose: where to return to. Defaults to spawn (3, 3) used by the
        # lab world / twin. Override via params if your spawn differs.
        self.declare_parameter('home_x',   3.0)
        self.declare_parameter('home_y',   3.0)
        self.declare_parameter('home_yaw', 0.0)

        gp = self.get_parameter
        self._odom_topic    = gp('odom_topic').value
        self._battery_topic = gp('battery_topic').value
        self._low_thresh    = float(gp('return_battery_percent').value)
        self._home_x        = float(gp('home_x').value)
        self._home_y        = float(gp('home_y').value)
        self._home_yaw      = float(gp('home_yaw').value)

        # ---- state ----
        self._x = self._y = 0.0
        self._battery_pct = 100.0
        self._state   = 'idle'          # idle | navigating | returning_home
        self._current: Optional[str] = None
        self._completed: list = []
        self._busy = threading.Lock()   # only one motion task at a time
        self._cancel_requested = False  # set by return_home / stop

        # ---- Nav2 ----
        self._nav = BasicNavigator()

        # ---- ROS I/O ----
        self.create_subscription(Odometry,     self._odom_topic,    self._odom_cb,    10)
        self.create_subscription(BatteryState, self._battery_topic, self._battery_cb, 10)
        self._status_pub = self.create_publisher(String, '/navigator/status', 10)
        self.create_service(Trigger, '/start_navigation', self._start_srv)
        self.create_service(Trigger, '/return_home',      self._return_home_srv)
        self.create_service(Trigger, '/stop_navigation',  self._stop_srv)
        self.create_service(Trigger, '/nav_status',       self._nav_status_srv)
        self.create_timer(3.0, self._broadcast)
        self.create_timer(float(gp('battery_check_period').value), self._battery_watch)

        self.get_logger().info(f'Navigator (Nav2 Simple Commander) started | {len(WAYPOINTS)} zones')
        for wp in WAYPOINTS:
            self.get_logger().info(f'  {wp["action"].upper():<11} {wp["name"]} at ({wp["x"]}, {wp["y"]})')
        self.get_logger().info(f'Return-home battery threshold: {self._low_thresh:.0f}%')

        # Wait for Nav2 to be up. If asked, seed AMCL with the home pose so we
        # don't need a manual "2D Pose Estimate" click (sim convenience).
        if bool(gp('set_initial_pose').value):
            self._nav.setInitialPose(self._make_pose(self._home_x, self._home_y, self._home_yaw))
        self.get_logger().info('Waiting for Nav2 to become active...')
        self._nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active. Call /start_navigation to begin.')

    # ---------------- callbacks ----------------
    def _odom_cb(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def _battery_cb(self, msg: BatteryState):
        # BatteryState.percentage is 0..1 on real TB3; some sources give 0..100.
        pct = msg.percentage
        if pct is not None and not math.isnan(pct):
            self._battery_pct = pct * 100.0 if pct <= 1.0 else pct

    def _battery_watch(self):
        if self._state == 'navigating' and self._battery_pct < self._low_thresh:
            self.get_logger().warn(
                f'[BATTERY] {self._battery_pct:.0f}% < {self._low_thresh:.0f}% '
                f'-> aborting zones, returning home')
            self._trigger_return_home()

    # ---------------- services ----------------
    def _start_srv(self, req, res):
        if self._state != 'idle':
            res.success = False; res.message = f'Busy ({self._state})'; return res
        self._cancel_requested = False
        threading.Thread(target=self._run_all, daemon=True).start()
        res.success = True
        res.message = f'Navigation started — {len(WAYPOINTS)} zones'
        return res

    def _return_home_srv(self, req, res):
        if self._state == 'returning_home':
            res.success = False; res.message = 'Already returning home'; return res
        self._trigger_return_home()
        res.success = True; res.message = 'Returning home'; return res

    def _stop_srv(self, req, res):
        self._cancel_requested = True
        self._nav.cancelTask()
        res.success = True; res.message = 'Navigation cancelled — going idle'; return res

    def _nav_status_srv(self, req, res):
        remaining = [w['name'] for w in WAYPOINTS if w['name'] not in self._completed]
        res.success = True
        res.message = (f'state={self._state}\ncurrent={self._current}\n'
                       f'battery={self._battery_pct:.0f}%\n'
                       f'completed={self._completed}\nremaining={remaining}\n'
                       f'position=({self._x:.2f}, {self._y:.2f})')
        return res

    # ---------------- motion ----------------
    def _trigger_return_home(self):
        """Cancel current Nav2 task and launch the go-home thread."""
        self._cancel_requested = True
        self._nav.cancelTask()
        threading.Thread(target=self._go_home, daemon=True).start()

    def _make_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self._nav.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation = yaw_to_quat(yaw)
        return p

    def _drive_to(self, x: float, y: float, yaw: float = 0.0, timeout_s: float = 120.0) -> bool:
        """Send one NavigateToPose goal and block until done/cancel/timeout."""
        self._nav.goToPose(self._make_pose(x, y, yaw))
        start = self.get_clock().now()
        while not self._nav.isTaskComplete():
            if self._cancel_requested:
                self._nav.cancelTask()
                return False
            if (self.get_clock().now() - start) > Duration(seconds=timeout_s):
                self.get_logger().warn('[NAV] goal timeout — cancelling')
                self._nav.cancelTask()
                return False
            # NOTE: do NOT rclpy.spin_once(self) here. main() already spins this
            # node via rclpy.spin(node), and BasicNavigator spins its own node
            # inside isTaskComplete(). Spinning the same global executor from two
            # threads is explicitly forbidden by rclpy and corrupts the wait set.
            time.sleep(0.1)
        return self._nav.getResult() == TaskResult.SUCCEEDED

    def _run_all(self):
        if not self._busy.acquire(blocking=False):
            return
        try:
            self._state = 'navigating'
            self._completed = []
            for wp in WAYPOINTS:
                if self._cancel_requested:
                    break
                self._current = wp['name']
                self.get_logger().info(f'[NAV] -> {wp["name"]} at ({wp["x"]}, {wp["y"]})')
                if self._drive_to(wp['x'], wp['y'], wp['yaw']):
                    self.get_logger().info(f'[ARRIVED] {wp["name"]} — {wp["action"].upper()}')
                    self._dwell(wp['pause_s'])  # let zone_monitor fire /farm_action
                    self._completed.append(wp['name'])
                else:
                    self.get_logger().warn(f'[SKIP] {wp["name"]} (cancelled or failed)')
            self.get_logger().info(
                f'[NAV] sequence done — {len(self._completed)}/{len(WAYPOINTS)} zones visited')
        finally:
            self._current = None
            # If a return-home was requested, _go_home owns the next state.
            if not self._cancel_requested:
                self._state = 'idle'
            self._busy.release()

    def _go_home(self):
        # Wait for any in-flight _run_all to release the lock.
        with self._busy:
            self._state = 'returning_home'
            self._current = 'home'
            self._cancel_requested = False  # fresh task
            self.get_logger().info(
                f'[HOME] -> ({self._home_x}, {self._home_y})')
            ok = self._drive_to(self._home_x, self._home_y, self._home_yaw, timeout_s=180.0)
            self.get_logger().info('[HOME] reached' if ok else '[HOME] failed/cancelled')
            self._current = None
            self._state = 'idle'

    def _dwell(self, seconds: float):
        # Wall-clock dwell so zone_monitor_node has time to fire /farm_action.
        # Callbacks keep flowing via the main rclpy.spin(node); see _drive_to note.
        end = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < end and not self._cancel_requested:
            time.sleep(0.1)

    # ---------------- status ----------------
    def _broadcast(self):
        msg = String()
        msg.data = (f'state={self._state} | current={self._current} | '
                    f'battery={self._battery_pct:.0f}% | completed={self._completed}')
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
