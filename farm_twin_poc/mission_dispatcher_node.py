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

Topics
  PUB /destination          — std_msgs/String, JSON: one waypoint dict
                              (name, x, y, yaw, action, pause_s)
  SUB /destination_reached  — std_msgs/String, JSON: {name, success}
  PUB /dispatcher/status    — std_msgs/String, human-readable progress

Services (std_srvs/Trigger)
  /start_navigation — start the tour from waypoint 0
  /stop_dispatch    — abandon the tour (does NOT cancel an in-flight
                      drive — for that use the navigator's
                      /stop_navigation or /return_home)
"""
import json
import threading

import rclpy
from rclpy.node import Node
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

        self._lock     = threading.Lock()
        self._running  = False
        self._index    = 0          # index of WAYPOINT currently in flight
        self._awaiting = None       # name we expect to see on /destination_reached

        self._dest_pub   = self.create_publisher(String, '/destination',        10)
        self._status_pub = self.create_publisher(String, '/dispatcher/status',  10)
        self.create_subscription(String, '/destination_reached', self._reached_cb, 10)
        self.create_service(Trigger, '/start_navigation', self._start_srv)
        self.create_service(Trigger, '/stop_dispatch',    self._stop_srv)

        self.create_timer(3.0, self._broadcast)

        self.get_logger().info(
            f'Mission dispatcher ready — {len(WAYPOINTS)} waypoints. '
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
            self._running = True
            self._index   = 0
        self.get_logger().info(
            f'[DISPATCH] mission started — {len(WAYPOINTS)} destinations')
        self._publish_current()
        res.success = True
        res.message = f'Mission started — {len(WAYPOINTS)} destinations'
        return res

    def _stop_srv(self, req, res):
        with self._lock:
            was_running = self._running
            self._running = False
            self._awaiting = None
        msg = ('Dispatch stopped' if was_running else 'Not running')
        self.get_logger().info(f'[DISPATCH] {msg}')
        res.success = True
        res.message = msg
        return res

    # ---------------- destination flow ----------------
    def _publish_current(self):
        with self._lock:
            if not self._running:
                return
            if self._index >= len(WAYPOINTS):
                self.get_logger().info(
                    f'[DISPATCH] sequence complete — '
                    f'{len(WAYPOINTS)} destinations reached')
                self._running  = False
                self._awaiting = None
                return
            wp = WAYPOINTS[self._index]
            self._awaiting = wp['name']
        msg = String()
        msg.data = json.dumps(wp)
        self.get_logger().info(
            f'[DISPATCH] sending {wp["name"]} '
            f'({self._index + 1}/{len(WAYPOINTS)})')
        self._dest_pub.publish(msg)

    def _reached_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'[DISPATCH] invalid /destination_reached JSON: {msg.data!r}')
            return
        name    = data.get('name')
        success = bool(data.get('success', False))

        with self._lock:
            if not self._running:
                return
            # Discard stale acks from a previous mission.
            if name != self._awaiting:
                self.get_logger().warn(
                    f'[DISPATCH] expected {self._awaiting!r}, '
                    f'got {name!r} — ignoring')
                return
            if not success:
                self.get_logger().warn(
                    f'[DISPATCH] {name} failed — aborting mission')
                self._running  = False
                self._awaiting = None
                return
            self._index += 1

        self.get_logger().info(f'[DISPATCH] reached {name}')
        self._publish_current()

    # ---------------- status ----------------
    def _broadcast(self):
        with self._lock:
            line = (f'running={self._running} | '
                    f'index={self._index}/{len(WAYPOINTS)} | '
                    f'awaiting={self._awaiting}')
        m = String(); m.data = line
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
