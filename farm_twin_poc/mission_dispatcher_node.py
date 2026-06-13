#!/usr/bin/env python3
"""
mission_dispatcher_node.py  —  Sequences WAYPOINTS into /destination
====================================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Owns the farm-zone tour plan: the WAYPOINTS list and the order in which
they're visited. Exposes /start_navigation (the same Trigger service
that used to live on navigator_node) and, on call, walks the list
one-at-a-time:

  - publish WAYPOINTS[i] as JSON on /destination
  - wait for /destination_reached confirming that name
  - on success: advance to WAYPOINTS[i+1]
  - on failure: log + stop the mission

Also watches /battery_state — when the level drops below
return_battery_percent (default 20 %) it stops the mission and calls
the navigator's /return_home service so the robot drives back to its
spawn pose.

Topics
  PUB /destination          — std_msgs/String, JSON: one waypoint dict
                              (name, x, y, yaw, action, pause_s)
  SUB /destination_reached  — std_msgs/String, JSON: {name, success}
  SUB /battery_state        — sensor_msgs/BatteryState — return-home trigger
  PUB /dispatcher/status    — std_msgs/String, JSON status payload

Services (std_srvs/Trigger)
  /start_navigation — start the tour from waypoint 0
  /stop_dispatch    — abandon the tour (does NOT cancel an in-flight
                      drive — for that use the navigator's
                      /stop_navigation or /return_home)

Client (std_srvs/Trigger)
  /return_home      — called on the navigator when battery drops below
                      the threshold
"""
import json
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger


# Aligned with FARM_ZONES in zone_monitor_node.py and the zone markers in
# worlds/new_world.world. Visit order: top-left spray → top-right fertilize
# → bottom-right fertilize → bottom-left spray (S-curve, shortest path).
WAYPOINTS = [
    {'name': 'spray_zone_A',     'x':  0.0, 'y':  0.4, 'yaw': 0.0, 'action': 'spray',     'pause_s': 2.0},
    {'name': 'fertilize_zone_B', 'x':  1.8, 'y':  0.4, 'yaw': 0.0, 'action': 'fertilize', 'pause_s': 2.0},
    {'name': 'fertilize_zone_D', 'x':  1.6, 'y': -3.0, 'yaw': 0.0, 'action': 'fertilize', 'pause_s': 2.0},
    {'name': 'spray_zone_C',     'x':  0.0, 'y': -3.0, 'yaw': 0.0, 'action': 'spray',     'pause_s': 2.0},
]


class MissionDispatcherNode(Node):

    def __init__(self):
        super().__init__('mission_dispatcher_node')

        self.declare_parameter('return_battery_percent',  20.0)
        # Hysteresis: don't re-arm the low-battery trigger until the
        # battery climbs at least this many points above the threshold
        # (e.g. a manual UI override goes to 50 %). Stops a battery
        # hovering near 20 % from spamming /return_home.
        self.declare_parameter('battery_hysteresis_percent', 5.0)
        gp = self.get_parameter
        self._low_thresh  = float(gp('return_battery_percent').value)
        self._hyst        = float(gp('battery_hysteresis_percent').value)

        self._lock      = threading.Lock()
        self._running   = False
        self._index     = 0          # index of WAYPOINT currently in flight
        self._awaiting  = None       # name we expect on /destination_reached
        self._completed: list[str] = []  # accumulates as acks arrive
        # "idle" | "running" | "complete" | "aborted" — what the UI shows.
        self._mission_state = 'idle'

        # Battery state + latch so we trigger at most once per drain cycle.
        self._battery_pct           = 100.0
        self._battery_seen          = False
        self._low_battery_triggered = False

        self._dest_pub   = self.create_publisher(String, '/destination',        10)
        self._status_pub = self.create_publisher(String, '/dispatcher/status',  10)
        self.create_subscription(String,       '/destination_reached', self._reached_cb, 10)
        self.create_subscription(BatteryState, '/battery_state',       self._battery_cb, 10)
        self.create_service(Trigger, '/start_navigation', self._start_srv)
        self.create_service(Trigger, '/stop_dispatch',    self._stop_srv)
        # Client we use to send the robot home when the battery is low.
        self._return_home_client = self.create_client(Trigger, '/return_home')

        self.create_timer(3.0, self._broadcast)

        self.get_logger().info(
            f'Mission dispatcher ready — {len(WAYPOINTS)} waypoints. '
            f'Return-home below {self._low_thresh:.0f}%. '
            f'Call /start_navigation to begin.')
        for i, wp in enumerate(WAYPOINTS):
            self.get_logger().info(
                f'  [{i}] {wp["action"].upper():<11} {wp["name"]} '
                f'at ({wp["x"]}, {wp["y"]})')

    # ---------------- services ----------------
    def _start_srv(self, req, res):
        with self._lock:
            if self._running:
                res.success = False
                res.message = (f'Already running '
                               f'({self._index+1}/{len(WAYPOINTS)} in flight)')
                return res
            if not WAYPOINTS:
                res.success = False
                res.message = 'No waypoints defined'
                return res
            # Block starting if we'd just turn around. The trigger latch
            # is also cleared so the next drain cycle re-arms.
            if self._battery_seen and self._battery_pct < self._low_thresh:
                res.success = False
                res.message = (f'Battery too low ({self._battery_pct:.0f}%) '
                               f'— charge above {self._low_thresh:.0f}% first')
                return res
            self._running               = True
            self._index                 = 0
            self._completed             = []
            self._mission_state         = 'running'
            self._low_battery_triggered = False
        self.get_logger().info(
            f'[DISPATCH] mission started — {len(WAYPOINTS)} destinations')
        self._publish_current()
        self._broadcast()
        res.success = True
        res.message = f'Mission started — {len(WAYPOINTS)} destinations'
        return res

    def _stop_srv(self, req, res):
        with self._lock:
            was_running = self._running
            self._running  = False
            self._awaiting = None
            if was_running:
                self._mission_state = 'aborted'
        msg = ('Dispatch stopped' if was_running else 'Not running')
        self.get_logger().info(f'[DISPATCH] {msg}')
        self._broadcast()
        res.success = True
        res.message = msg
        return res

    # ---------------- destination flow ----------------
    def _publish_current(self):
        wp = None
        with self._lock:
            if not self._running:
                return
            if self._index >= len(WAYPOINTS):
                self.get_logger().info(
                    f'[DISPATCH] sequence complete — '
                    f'{len(WAYPOINTS)} destinations reached')
                self._running       = False
                self._awaiting      = None
                self._mission_state = 'complete'
            else:
                wp = WAYPOINTS[self._index]
                self._awaiting = wp['name']
        if wp is None:
            self._broadcast()
            return
        msg = String()
        msg.data = json.dumps(wp)
        self.get_logger().info(
            f'[DISPATCH] sending {wp["name"]} '
            f'({self._index + 1}/{len(WAYPOINTS)})')
        self._dest_pub.publish(msg)
        self._broadcast()

    def _reached_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'[DISPATCH] invalid /destination_reached JSON: {msg.data!r}')
            return
        name    = data.get('name')
        success = bool(data.get('success', False))

        advance = False
        with self._lock:
            if not self._running:
                return
            if name != self._awaiting:
                self.get_logger().warn(
                    f'[DISPATCH] expected {self._awaiting!r}, '
                    f'got {name!r} — ignoring')
                return
            if not success:
                self.get_logger().warn(
                    f'[DISPATCH] {name} failed — aborting mission')
                self._running       = False
                self._awaiting      = None
                self._mission_state = 'aborted'
            else:
                self._completed.append(name)
                self._index += 1
                advance = True

        self.get_logger().info(f'[DISPATCH] reached {name}')
        if advance:
            self._publish_current()
        else:
            self._broadcast()

    # ---------------- battery watch ----------------
    def _battery_cb(self, msg: BatteryState) -> None:
        # BatteryState.percentage is 0..1 per ROS convention, but some
        # publishers report 0..100. Accept either, store as 0..100.
        pct = msg.percentage
        if pct is None or math.isnan(pct):
            return
        val = pct * 100.0 if pct <= 1.0 else pct
        # Gazebo bridges sometimes publish 0 at startup before the sim
        # is initialised — don't treat that as a real low-battery event.
        if val <= 0.0 and not self._battery_seen:
            return
        self._battery_seen = True
        self._battery_pct  = val

        # Re-arm the trigger once battery rises back above threshold +
        # hysteresis (e.g. a manual UI override bumps it to 80 %).
        if val > self._low_thresh + self._hyst:
            self._low_battery_triggered = False

        if val < self._low_thresh and not self._low_battery_triggered:
            self._low_battery_triggered = True
            self.get_logger().warn(
                f'[DISPATCH] battery {val:.0f}% < {self._low_thresh:.0f}% '
                f'— stopping mission and returning home')
            with self._lock:
                if self._running:
                    self._running       = False
                    self._awaiting      = None
                    self._mission_state = 'aborted'
            self._call_return_home()
            self._broadcast()

    def _call_return_home(self) -> None:
        """Fire /return_home asynchronously — never block this callback."""
        if not self._return_home_client.service_is_ready():
            # Service may not be up yet at boot. Brief retry in a thread
            # so we don't stall the executor.
            threading.Thread(
                target=self._wait_then_call_return_home, daemon=True).start()
            return
        self._return_home_client.call_async(Trigger.Request())

    def _wait_then_call_return_home(self) -> None:
        if self._return_home_client.wait_for_service(timeout_sec=3.0):
            self._return_home_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn(
                '[DISPATCH] /return_home not available — robot will not move home')

    # ---------------- status ----------------
    def _broadcast(self):
        # JSON so the web UI can render it structurally. web_server_node's
        # _string_to_dict() parses this back out as the `json` field.
        with self._lock:
            current_name = (WAYPOINTS[self._index]['name']
                            if self._running and self._index < len(WAYPOINTS)
                            else None)
            payload = {
                'stamp':     self.get_clock().now().nanoseconds * 1e-9,
                'state':     self._mission_state,
                'running':   self._running,
                'index':     self._index,
                'total':     len(WAYPOINTS),
                'current':   current_name,
                'awaiting':  self._awaiting,
                'completed': list(self._completed),
                # Battery snapshot + latch so the UI can flag the
                # low-battery return-home without re-subscribing to
                # /battery_state itself.
                'battery_pct':  (round(self._battery_pct, 1)
                                 if self._battery_seen else None),
                'low_battery':  self._low_battery_triggered,
                # Static list — sent on every tick so the UI doesn't need a
                # separate config endpoint. Tiny payload (4 entries).
                'waypoints': [
                    {'name': w['name'], 'action': w['action']}
                    for w in WAYPOINTS
                ],
            }
        m = String(); m.data = json.dumps(payload)
        self._status_pub.publish(m)


def main():
    rclpy.init()
    node = MissionDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
