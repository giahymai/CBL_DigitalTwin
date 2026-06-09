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
from rclpy.executors import SingleThreadedExecutor

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Aligned with FARM_ZONES in zone_monitor_node.py. yaw is the heading the robot
# should face on arrival (radians); it does not affect zone detection.
# Spray (red) zones on the top row (y=2.7), fertilize (green) on the bottom row
# (y=0.7). Robot spawns top-left (3,3): visits C (top-left, near spawn) → A
# (top-right) → D (bottom-right, directly below A) → B (bottom-left).
WAYPOINTS = [
    {'name': 'spray_zone_C',     'x': 3.5, 'y': 2.7, 'yaw': 0.0, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'spray_zone_A',     'x': 0.5, 'y': 2.7, 'yaw': 0.0, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'fertilize_zone_D', 'x': 0.5, 'y': 0.7, 'yaw': 0.0, 'action': 'fertilize', 'pause_s': 2.0},
    {'name': 'fertilize_zone_B', 'x': 3.5, 'y': 0.7, 'yaw': 0.0, 'action': 'fertilize', 'pause_s': 2.0},
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
        # Spray/fertilize signal: rotate in place at each zone. Done by directly
        # commanding /cmd_vel (yaw-tracked) rather than Nav2's spin behaviour,
        # which aborts near walls when its collision look-ahead trips on the
        # inflation layer (zones sit close to walls).
        self.declare_parameter('spin_speed',     1.2)        # rad/s (faster spray/fertilize spin)
        self.declare_parameter('spin_cmd_topic', '/cmd_vel')
        # Arrival handling (map frame). We let Nav2 drive the robot close to the
        # zone CENTRE rather than bailing out the moment we touch the trigger
        # radius (that left the robot parked off-centre). Two early exits keep
        # the tour from hanging on a slow sim:
        #   arrival_radius — close enough to the centre to call it arrived.
        #   stuck_time     — if the robot stops moving (< stuck_move) for this
        #                    long while Nav2 keeps trying, accept the current
        #                    spot and move on instead of waiting out the timeout.
        self.declare_parameter('arrival_radius', 0.18)
        self.declare_parameter('stuck_time',     8.0)
        self.declare_parameter('stuck_move',     0.05)
        self.declare_parameter('global_frame',   'map')
        self.declare_parameter('robot_frame',    'base_link')

        gp = self.get_parameter
        self._odom_topic     = gp('odom_topic').value
        self._battery_topic  = gp('battery_topic').value
        self._low_thresh     = float(gp('return_battery_percent').value)
        self._home_x         = float(gp('home_x').value)
        self._home_y         = float(gp('home_y').value)
        self._home_yaw       = float(gp('home_yaw').value)
        self._spin_speed     = float(gp('spin_speed').value)
        self._arrival_radius = float(gp('arrival_radius').value)
        self._stuck_time     = float(gp('stuck_time').value)
        self._stuck_move     = float(gp('stuck_move').value)
        self._global_frame   = gp('global_frame').value
        self._robot_frame    = gp('robot_frame').value

        # ---- state ----
        self._x = self._y = self._yaw = 0.0
        self._battery_pct = 100.0
        self._battery_seen = False      # ignore battery until a plausible (>0) reading
        self._state   = 'idle'          # idle | navigating | returning_home
        self._current: Optional[str] = None
        self._completed: list = []
        self._busy = threading.Lock()   # only one motion task at a time
        self._cancel_requested = False  # set by return_home / stop

        # ---- Nav2 ----
        self._nav = BasicNavigator()

        # TF, to read the robot pose in the map frame (for arrival_radius).
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- ROS I/O ----
        self.create_subscription(Odometry,     self._odom_topic,    self._odom_cb,    10)
        self.create_subscription(BatteryState, self._battery_topic, self._battery_cb, 10)
        self._status_pub = self.create_publisher(String, '/navigator/status', 10)
        self._cmd_pub    = self.create_publisher(
            TwistStamped, gp('spin_cmd_topic').value, 10)
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
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z))

    def _battery_cb(self, msg: BatteryState):
        # BatteryState.percentage is 0..1 on real TB3; some sources give 0..100.
        pct = msg.percentage
        if pct is not None and not math.isnan(pct):
            val = pct * 100.0 if pct <= 1.0 else pct
            self._battery_pct = val
            # In Gazebo a battery topic often publishes 0 (uninitialised). Treat
            # only a >0 reading as real, so a spurious 0% can't trigger a bogus
            # return-home that aborts the zone tour after the first goal.
            if val > 0.0:
                self._battery_seen = True

    def _battery_watch(self):
        if (self._battery_seen and self._state == 'navigating'
                and self._battery_pct < self._low_thresh):
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

    def _map_xy(self):
        """Robot (x, y) in the map frame via TF, or None if not available yet."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _drive_to(self, x: float, y: float, yaw: float = 0.0, timeout_s: float = 120.0) -> bool:
        """Send one NavigateToPose goal and block until done/cancel/timeout.
        Let Nav2 drive close to the zone CENTRE; exit early only when:
          - within arrival_radius of the centre (arrived, nicely centred), or
          - the robot hasn't moved (> stuck_move) for stuck_time while Nav2 keeps
            trying (stuck on a slow sim) — accept the spot and move on so the
            tour doesn't hang."""
        self._nav.goToPose(self._make_pose(x, y, yaw))
        start       = self.get_clock().now()
        last_pos    = self._map_xy()
        last_move_t = time.monotonic()
        while not self._nav.isTaskComplete():
            if self._cancel_requested:
                self._nav.cancelTask()
                return False
            pos = self._map_xy()
            if pos is not None:
                if math.hypot(x - pos[0], y - pos[1]) <= self._arrival_radius:
                    self._nav.cancelTask()
                    return True
                if last_pos is None or math.hypot(pos[0] - last_pos[0],
                                                  pos[1] - last_pos[1]) > self._stuck_move:
                    last_pos, last_move_t = pos, time.monotonic()
                elif time.monotonic() - last_move_t > self._stuck_time:
                    d = math.hypot(x - pos[0], y - pos[1])
                    self.get_logger().warn(f'[NAV] stuck {d:.2f} m from centre — moving on')
                    self._nav.cancelTask()
                    return True
            if (self.get_clock().now() - start) > Duration(seconds=timeout_s):
                self.get_logger().warn('[NAV] goal timeout — cancelling')
                self._nav.cancelTask()
                return False
            # NOTE: do NOT spin our node here. main() spins it on its own
            # dedicated executor, and isTaskComplete() spins the Nav2 node on the
            # GLOBAL executor. Keeping the two executors separate is what avoids
            # the "Executor is already spinning" clash (see main()).
            time.sleep(0.1)
        result = self._nav.getResult()
        if result != TaskResult.SUCCEEDED:
            self.get_logger().warn(f'[NAV] goal ended with result={result}')
        return result == TaskResult.SUCCEEDED

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
                    self._spin_action()  # 360° spin = spray/fertilize signal + dwell
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

    def _spin_action(self, revolutions: float = 1.0):
        """Rotate ~360° in place to signal the spray/fertilize action.

        Commands a steady angular velocity straight to /cmd_vel and tracks odom
        yaw until a full turn completes, so it ALWAYS spins. (Nav2's own spin
        behaviour was unreliable here: its collision look-ahead aborts near
        walls, where the inflation layer flags the cell, so zones close to a
        wall never spun.) linear.x stays 0 and the Nav2 goal is already complete,
        so nothing else is driving the robot. Also keeps the robot on the zone
        long enough for zone_monitor_node to fire /farm_action."""
        target      = revolutions * 2.0 * math.pi
        accumulated = 0.0
        last_yaw    = self._yaw
        t0          = time.monotonic()
        while accumulated < target and not self._cancel_requested:
            if time.monotonic() - t0 > 25.0:          # safety timeout
                self.get_logger().warn('[SPIN] timeout')
                break
            accumulated += abs(self._wrap(self._yaw - last_yaw))
            last_yaw     = self._yaw
            cmd = TwistStamped()
            cmd.header.stamp    = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.angular.z = self._spin_speed
            self._cmd_pub.publish(cmd)
            time.sleep(0.05)
        stop = TwistStamped()
        stop.header.stamp    = self.get_clock().now().to_msg()
        stop.header.frame_id = 'base_link'
        self._cmd_pub.publish(stop)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    # ---------------- status ----------------
    def _broadcast(self):
        msg = String()
        msg.data = (f'state={self._state} | current={self._current} | '
                    f'battery={self._battery_pct:.0f}% | completed={self._completed}')
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavigatorNode()
    # Spin our node on a DEDICATED executor, NOT the global one. nav2_simple_
    # commander's goToPose()/isTaskComplete() internally call
    # rclpy.spin_until_future_complete(...), which spins the GLOBAL executor. If
    # main() also did rclpy.spin(node) (the global executor), the two clash with
    # "RuntimeError: Executor is already spinning" and the navigation thread dies
    # right after sending the first goal — so only one zone is ever visited.
    # Giving our node its own executor leaves the global one free for Nav2.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
