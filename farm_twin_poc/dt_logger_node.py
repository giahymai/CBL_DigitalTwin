#!/usr/bin/env python3
"""
dt_logger_node.py  —  Node 4: Digital Entity Logger
====================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Core Digital Entity. Receives farm action events, maintains
a timestamped action log, broadcasts status every 5 seconds.

Topics:
  SUB /farm_action — events from zone_monitor_node
  SUB /odom        — robot position
  PUB /dt/status   — DE state broadcast (every 5 s)

Services:
  /get_dt_status — current summary
  /get_dt_log    — full action history
"""
import json
from datetime import datetime
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DTLoggerNode(Node):

    def __init__(self):
        super().__init__('dt_logger_node')
        self._log          = []
        self._spray_count  = 0
        self._fert_count   = 0
        self._x = self._y  = 0.0
        self._zones        = set()

        self.create_subscription(String,   '/farm_action', self._action_cb, 10)
        self.create_subscription(Odometry, '/odom',        self._odom_cb,   10)
        self._status_pub = self.create_publisher(String, '/dt/status', 10)
        self.create_service(Trigger, '/get_dt_status', self._status_srv)
        self.create_service(Trigger, '/get_dt_log',    self._log_srv)
        self.create_timer(5.0, self._broadcast)

        self.get_logger().info('Digital Twin Logger (Digital Entity) started')
        self.get_logger().info(
            'Query: ros2 service call /get_dt_status std_srvs/srv/Trigger'
        )

    def _odom_cb(self, msg):
        self._x, self._y = msg.pose.pose.position.x, msg.pose.pose.position.y

    def _action_cb(self, msg):
        try:
            event = json.loads(msg.data)
        except Exception:
            return
        event['logged_at'] = datetime.now().isoformat()
        self._log.append(event)
        self._zones.add(event['zone'])
        if event['action'] == 'spray':
            self._spray_count += 1
        elif event['action'] == 'fertilize':
            self._fert_count += 1
        self.get_logger().info(
            f'[DE LOG] {event["action"].upper()} | zone={event["zone"]} | '
            f'spray={self._spray_count}  fertilize={self._fert_count}'
        )
        self._broadcast()

    def _broadcast(self):
        msg = String(); msg.data = json.dumps(self._build())
        self._status_pub.publish(msg)

    def _build(self):
        return {
            'position':          {'x': round(self._x, 3), 'y': round(self._y, 3)},
            'spray_actions':     self._spray_count,
            'fertilize_actions': self._fert_count,
            'total_actions':     len(self._log),
            'zones_covered':     sorted(list(self._zones)),
            'last_action':       self._log[-1] if self._log else None,
        }

    def _status_srv(self, req, res):
        res.success = True; res.message = json.dumps(self._build(), indent=2); return res

    def _log_srv(self, req, res):
        res.success = True; res.message = json.dumps(self._log, indent=2); return res


def main():
    rclpy.init()
    node = DTLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
