#!/usr/bin/env python3
"""
navigator_node.py  —  Per-destination Nav2 driver
==================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Listens to /destination and drives the robot to whatever pose arrives.
After arriving it spins 360° (spray/fertilize signal), settles for a few
seconds so AMCL re-converges, then publishes /destination_reached so the
mission dispatcher can send the next goal.

This node owns NOTHING about the full mission — no WAYPOINTS list, no
sequence logic. It's a one-shot worker driven by a topic. The mission
plan (which destinations, in what order, when to start) lives in
mission_dispatcher_node.py.

Topics
  SUB  /destination          — std_msgs/String, JSON: {"name", "x", "y",
                               "yaw", "action", "pause_s"}. Anything
                               arriving here starts a drive immediately
                               (unless a previous one is still running,
                               in which case the new one is dropped).
  PUB  /destination_reached  — std_msgs/String, JSON: {"name", "success"}
                               Sent exactly once per processed destination.
  SUB  <odom_topic>          — current pose (default /odom)
  SUB  <battery_topic>       — sensor_msgs/BatteryState
  PUB  /navigator/status     — human-readable progress (every 3 s)

Services (std_srvs/Trigger)
  /return_home      — cancel current drive + drive to home pose now
  /stop_navigation  — cancel current drive, go idle
  /nav_status       — query state

Note: /start_navigation USED to live here. It is now on
mission_dispatcher_node — that's the new entry point for "drive the whole
zone tour".
"""
import json
import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


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
        self.declare_parameter('set_initial_pose',       False)
        # Home pose for /return_home. Defaults to the spawn used by the
        # current launch (1.5, -2.0). Override via params if you change the
        # spawn.
        self.declare_parameter('home_x',   1.5)
        self.declare_parameter('home_y',  -2.0)
        self.declare_parameter('home_yaw', 0.0)
        # Spray/fertilize signal: rotate in place at each destination.
        self.declare_parameter('spin_speed',     1.2)
        self.declare_parameter('settle_s',       5.0)
        self.declare_parameter('spin_cmd_topic', '/cmd_vel')
        # Arrival handling (map frame).
        self.declare_parameter('arrival_radius', 0.18)
        self.declare_parameter('stuck_time',     8.0)
        self.declare_parameter('stuck_move',     0.05)
        self.declare_parameter('global_frame',   'map')
        self.declare_parameter('robot_frame',    'base_link')

        gp = self.get_parameter
        self._odom_topic     = gp('odom_topic').value
        self._home_x         = float(gp('home_x').value)
        self._home_y         = float(gp('home_y').value)
        self._home_yaw       = float(gp('home_yaw').value)
        self._spin_speed     = float(gp('spin_speed').value)
        self._settle_s       = float(gp('settle_s').value)
        self._arrival_radius = float(gp('arrival_radius').value)
        self._stuck_time     = float(gp('stuck_time').value)
        self._stuck_move     = float(gp('stuck_move').value)
        self._global_frame   = gp('global_frame').value
        self._robot_frame    = gp('robot_frame').value

        # ---- state ----
        self._x = self._y = self._yaw = 0.0
        # idle | navigating | spraying | fertilizing | returning_home
        # spraying / fertilizing are reported while the spin-action is
        # in progress at a destination — the UI can render distinct
        # colours per phase without parsing the waypoint action.
        self._state   = 'idle'
        self._current: Optional[str] = None
        self._completed: list = []      # destinations finished since startup
        self._busy = threading.Lock()
        self._cancel_requested = False

        # ---- Nav2 ----
        self._nav = BasicNavigator()

        # TF for map-frame arrival checks.
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- ROS I/O ----
        self.create_subscription(Odometry,     self._odom_topic,    self._odom_cb,    10)
        self.create_subscription(String,       '/destination',      self._destination_cb, 10)
        self._status_pub   = self.create_publisher(String, '/navigator/status',     10)
        self._reached_pub  = self.create_publisher(String, '/destination_reached',  10)
        self._cmd_pub      = self.create_publisher(
            TwistStamped, gp('spin_cmd_topic').value, 10)
        self.create_service(Trigger, '/return_home',      self._return_home_srv)
        self.create_service(Trigger, '/stop_navigation',  self._stop_srv)
        self.create_service(Trigger, '/nav_status',       self._nav_status_srv)
        self.create_timer(3.0, self._broadcast)

        self.get_logger().info(
            'Navigator started — listening on /destination '
            '(publish a JSON pose to drive there)')
        self.get_logger().info(
            'Low-battery return-home is owned by mission_dispatcher_node.')

        if bool(gp('set_initial_pose').value):
            self._nav.setInitialPose(self._make_pose(self._home_x, self._home_y, self._home_yaw))
        self.get_logger().info('Waiting for Nav2 to become active...')
        self._nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active. Awaiting /destination messages.')

    # ---------------- sensor callbacks ----------------
    def _odom_cb(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z))

    # ---------------- /destination handler ----------------
    def _destination_cb(self, msg: String):
        try:
            dest = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'[DEST] invalid JSON: {msg.data!r}')
            return
        for k in ('name', 'x', 'y'):
            if k not in dest:
                self.get_logger().warn(f'[DEST] missing required field "{k}"')
                return
        # Process in a thread so we don't block the executor / subscriber.
        threading.Thread(
            target=self._handle_destination, args=(dest,), daemon=True).start()

    def _handle_destination(self, dest: dict):
        # If another destination is already running, drop this one. The
        # dispatcher contract is "send next only after /destination_reached",
        # so this should never fire in normal operation.
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn(
                f'[DEST] busy with {self._current!r}, ignoring {dest.get("name")!r}')
            return
        success = False
        try:
            self._cancel_requested = False
            self._state   = 'navigating'
            self._current = dest['name']
            self._publish_status()   # don't wait for the 3 s heartbeat
            x      = float(dest['x'])
            y      = float(dest['y'])
            yaw    = float(dest.get('yaw', 0.0))
            action = str(dest.get('action', ''))

            self.get_logger().info(f'[NAV] -> {dest["name"]} at ({x:.2f}, {y:.2f})')
            if self._drive_to(x, y, yaw):
                self.get_logger().info(
                    f'[ARRIVED] {dest["name"]}' +
                    (f' — {action.upper()}' if action else ''))
                self._spin_action(action=action)
                self._completed.append(dest['name'])
                success = True
            else:
                self.get_logger().warn(
                    f'[SKIP] {dest["name"]} (cancelled or failed)')
        finally:
            self._current = None
            if not self._cancel_requested:
                self._state = 'idle'
            self._publish_status()
            self._busy.release()
            # Always tell the dispatcher we're done with this one, success or
            # not — otherwise the mission would stall waiting for an ack.
            done = String()
            done.data = json.dumps({'name': dest['name'], 'success': success})
            self._reached_pub.publish(done)

    # ---------------- services ----------------
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
        res.success = True
        res.message = (f'state={self._state}\ncurrent={self._current}\n'
                       f'completed={self._completed}\n'
                       f'position=({self._x:.2f}, {self._y:.2f})')
        return res

    # ---------------- motion ----------------
    def _trigger_return_home(self):
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
        try:
            t = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _drive_to(self, x: float, y: float, yaw: float = 0.0, timeout_s: float = 120.0) -> bool:
        """One NavigateToPose goal, blocking until done/cancel/timeout."""
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
            time.sleep(0.1)
        result = self._nav.getResult()
        if result != TaskResult.SUCCEEDED:
            self.get_logger().warn(f'[NAV] goal ended with result={result}')
        return result == TaskResult.SUCCEEDED

    def _go_home(self):
        with self._busy:
            self._state = 'returning_home'
            self._current = 'home'
            self._cancel_requested = False
            self._publish_status()
            self.get_logger().info(
                f'[HOME] -> ({self._home_x}, {self._home_y})')
            ok = self._drive_to(self._home_x, self._home_y, self._home_yaw, timeout_s=180.0)
            self.get_logger().info('[HOME] reached' if ok else '[HOME] failed/cancelled')
            self._current = None
            self._state = 'idle'
            self._publish_status()

    def _spin_action(self, action: str = '', revolutions: float = 1.0):
        """Stand still for ~5 s to simulate the spray/fertilize action.

        The robot used to rotate ~360° here to visualise the action, but
        the simulated effect doesn't require motion — holding position
        while we publish zero twist is enough. While the hold runs we
        also advertise the action as the navigator state so the UI shows
        "Spraying" / "Fertilizing" instead of a generic "Navigating".
        """
        if action == 'spray':
            self._state = 'spraying'
        elif action == 'fertilize':
            self._state = 'fertilizing'
        # Push the spraying/fertilizing state out now so the Twin Status
        # badge flips in step with the action appearing in the history.
        self._publish_status()

        # Stand still for _settle_s seconds (default 5.0). Publishing
        # zero twist keeps any prior cmd_vel from carrying over and also
        # gives AMCL / costmaps clean LiDAR sweeps before the next goal.
        hold_s = self._settle_s if self._settle_s > 0.0 else 5.0
        self.get_logger().info(f'[ACTION] holding still {hold_s:.1f}s')
        t_end = time.monotonic() + hold_s
        while time.monotonic() < t_end and not self._cancel_requested:
            z = TwistStamped()
            z.header.stamp    = self.get_clock().now().to_msg()
            z.header.frame_id = 'base_link'
            self._cmd_pub.publish(z)
            time.sleep(0.1)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    # ---------------- status ----------------
    def _broadcast(self):
        # Periodic heartbeat (every 3 s) so position keeps refreshing even
        # when the state is static. State *transitions* publish immediately
        # via _publish_status() so the Twin Status badge never lags the
        # action history by up to a full broadcast interval.
        self._publish_status()

    def _publish_status(self):
        # JSON so the web UI can render it structurally. web_server_node's
        # _string_to_dict() parses this back out under the `json` field.
        payload = {
            'stamp':     self.get_clock().now().nanoseconds * 1e-9,
            'state':     self._state,
            'current':   self._current,
            'completed': list(self._completed),
            'position':  {'x':   round(self._x,   3),
                          'y':   round(self._y,   3),
                          'yaw': round(self._yaw, 3)},
        }
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False)
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavigatorNode()
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
