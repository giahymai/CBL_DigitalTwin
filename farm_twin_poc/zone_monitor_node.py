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
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

FARM_ZONES = [
    {'name': 'spray_zone_A',     'x':  0.0, 'y':  2.0, 'radius': 0.2, 'action': 'spray',
     'description': 'Striga-infested zone — targeted herbicide spray'},
    {'name': 'fertilize_zone_B', 'x':  0.0, 'y':  1.0, 'radius': 0.2, 'action': 'fertilize',
     'description': 'Low-NPK zone — variable-rate fertilizer application'},
    {'name': 'spray_zone_C',     'x': -1.0, 'y':  0.0, 'radius': 0.2, 'action': 'spray',
     'description': 'Striga-infested zone — targeted herbicide spray'},
    {'name': 'fertilize_zone_D', 'x':  0.0, 'y': -1.0, 'radius': 0.2, 'action': 'fertilize',
     'description': 'Low-NPK zone — variable-rate fertilizer application'},
]


class ZoneMonitorNode(Node):

    def __init__(self):
        super().__init__('zone_monitor_node')
        self._x = 0.0
        self._y = 0.0
        self._odom_ok = False
        self._in_zone     = {z['name']: False for z in FARM_ZONES}
        self._trigger_cnt = {z['name']: 0     for z in FARM_ZONES}

        self.declare_parameter('odom_topic', '/sim/odom')
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.get_logger().info(f'SUB odom: {odom_topic}')
        self._pub = self.create_publisher(String, '/farm_action', 10)
        self.create_service(Trigger, '/get_zone_status', self._status_srv)

        self.get_logger().info(f'Zone Monitor Node started | {len(FARM_ZONES)} zones loaded')
        for z in FARM_ZONES:
            self.get_logger().info(
                f'  [{z["action"].upper():<11}] {z["name"]:<20} '
                f'at ({z["x"]:+.1f}, {z["y"]:+.1f})  r={z["radius"]} m'
            )

    def _odom_cb(self, msg):
        self._x, self._y = msg.pose.pose.position.x, msg.pose.pose.position.y
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