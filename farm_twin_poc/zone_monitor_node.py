#!/usr/bin/env python3
"""
zone_monitor_node.py  —  Node 3: Farm Zone Monitor
===================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Monitors robot position and detects entry into farm zones.
Publishes a spray/fertilize action when robot enters a zone.

Topics:
  SUB /odom         — robot position from Physical Entity
  PUB /farm_action  — JSON action event to Digital Entity

Service:
  /get_zone_status  — query zone trigger history

ADJUST FARM_ZONES before lab session:
  Set (x, y) to match physical markers placed in the lab room.
  Coordinates are metres from robot start position (odom frame).
"""
import json
import math
import time
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

FARM_ZONES = [
    {'name': 'spray_zone_A',     'x':  0.0, 'y':  0.4, 'radius': 0.35, 'action': 'spray',
     'description': 'Striga-infested zone — targeted herbicide spray'},
    {'name': 'fertilize_zone_B', 'x':  1.8, 'y':  0.4, 'radius': 0.35, 'action': 'fertilize',
     'description': 'Low-NPK zone — variable-rate fertilizer application'},
    {'name': 'fertilize_zone_D', 'x':  1.6, 'y': -3.0, 'radius': 0.35, 'action': 'fertilize',
     'description': 'Low-NPK zone — variable-rate fertilizer application'},
    {'name': 'spray_zone_C',     'x':  0.0, 'y': -3.0, 'radius': 0.35, 'action': 'spray',
     'description': 'Striga-infested zone — targeted herbicide spray'},
]


class ZoneMonitorNode(Node):

    def __init__(self):
        super().__init__('zone_monitor_node')
        self._x = 0.0
        self._y = 0.0
        self._odom_ok = False
        self._in_zone     = {z['name']: False for z in FARM_ZONES}
        self._trigger_cnt = {z['name']: 0     for z in FARM_ZONES}

        # Position source. FARM_ZONES coords live in the same frame as the
        # Gazebo zone tiles (the world/map frame).
        #   'odom' — read /odom directly. Correct for reactive + teleop, where
        #            there is no map/AMCL and /odom is anchored at spawn ≈ world.
        #   'tf'   — look up <global_frame>-><robot_frame> (map->base_link).
        #            REQUIRED under Nav2: /odom drifts and AMCL corrects it via a
        #            map->odom transform, so /odom no longer matches the tiles.
        #            The robot reaches the tile in the map frame but /odom reads
        #            a different number, so zone detection on /odom never fires.
        self.declare_parameter('position_source', 'odom')
        self.declare_parameter('odom_topic',   '/sim/odom')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame',  'base_link')
        self._source = self.get_parameter('position_source').value

        self._pub = self.create_publisher(String, '/farm_action', 10)
        self.create_service(Trigger, '/get_zone_status', self._status_srv)

        if self._source == 'tf':
            self._global_frame = self.get_parameter('global_frame').value
            self._robot_frame  = self.get_parameter('robot_frame').value
            self._tf_buffer    = Buffer()
            self._tf_listener  = TransformListener(self._tf_buffer, self)
            self.create_timer(0.1, self._tf_cb)
            self.get_logger().info(
                f'Position source: TF {self._global_frame} -> {self._robot_frame}')
        else:
            odom_topic = self.get_parameter('odom_topic').value
            self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
            self.get_logger().info(f'Position source: odom topic {odom_topic}')

        # Optional 360° spin on zone entry — for TELEOP demos ONLY. The Nav2
        # navigator already spins on arrival, so leave this False whenever the
        # navigator is running, or the two will both command motion and fight.
        self.declare_parameter('spin_on_entry',  False)
        self.declare_parameter('spin_speed',     0.6)        # rad/s
        self.declare_parameter('spin_cmd_topic', '/cmd_vel_raw')
        self._spin_on_entry = bool(self.get_parameter('spin_on_entry').value)
        self._spin_speed    = float(self.get_parameter('spin_speed').value)
        self._spinning      = False
        self._spin_pub      = None
        if self._spin_on_entry:
            self._spin_pub = self.create_publisher(
                TwistStamped, self.get_parameter('spin_cmd_topic').value, 10)
            self.get_logger().warn(
                'spin_on_entry ON → robot spins 360° at each zone. '
                'Use in TELEOP only; do NOT run with a navigator.')

        self.get_logger().info(f'Zone Monitor Node started | {len(FARM_ZONES)} zones loaded')
        for z in FARM_ZONES:
            self.get_logger().info(
                f'  [{z["action"].upper():<11}] {z["name"]:<20} '
                f'at ({z["x"]:+.1f}, {z["y"]:+.1f})  r={z["radius"]} m'
            )

    def _odom_cb(self, msg):
        self._update_position(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _tf_cb(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return  # transform not available yet — wait for the next tick
        self._update_position(t.transform.translation.x, t.transform.translation.y)

    def _update_position(self, x, y):
        self._x, self._y = x, y
        self._odom_ok = True
        for zone in FARM_ZONES:
            dist = math.sqrt((self._x - zone['x'])**2 + (self._y - zone['y'])**2)
            inside = dist < zone['radius']
            if inside and not self._in_zone[zone['name']]:
                self._in_zone[zone['name']] = True
                self._trigger_cnt[zone['name']] += 1
                self._publish(zone, dist)
            elif not inside and self._in_zone[zone['name']]:
                self._in_zone[zone['name']] = False

    def _publish(self, zone, dist):
        event = {
            'zone': zone['name'], 'action': zone['action'],
            'description': zone['description'],
            'position': {'x': round(self._x, 3), 'y': round(self._y, 3)},
            'distance_m': round(dist, 3),
            'trigger_count': self._trigger_cnt[zone['name']],
        }
        msg = String(); msg.data = json.dumps(event)
        self._pub.publish(msg)
        self.get_logger().info(
            f'[ZONE ENTRY] {zone["action"].upper()} → {zone["name"]} | '
            f'pos=({self._x:.2f},{self._y:.2f}) dist={dist:.2f}m'
        )
        if self._spin_on_entry and not self._spinning:
            threading.Thread(target=self._spin_360, daemon=True).start()

    def _spin_360(self):
        """Rotate ~360° on the spot to signal spraying/fertilizing."""
        self._spinning = True
        duration = 2.0 * math.pi / max(0.1, self._spin_speed)
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            cmd = TwistStamped()
            cmd.header.stamp    = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.angular.z = self._spin_speed
            self._spin_pub.publish(cmd)
            time.sleep(0.05)
        stop = TwistStamped()
        stop.header.stamp    = self.get_clock().now().to_msg()
        stop.header.frame_id = 'base_link'
        self._spin_pub.publish(stop)
        self._spinning = False

    def _status_srv(self, req, res):
        res.success = True
        res.message = json.dumps({
            'position': {'x': round(self._x, 3), 'y': round(self._y, 3)},
            'odom_active': self._odom_ok,
            'zones': [{'name': z['name'], 'action': z['action'],
                       'inside': self._in_zone[z['name']],
                       'triggered': self._trigger_cnt[z['name']]}
                      for z in FARM_ZONES],
            'total_actions': sum(self._trigger_cnt.values()),
        }, indent=2)
        return res


def main():
    rclpy.init()
    node = ZoneMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()