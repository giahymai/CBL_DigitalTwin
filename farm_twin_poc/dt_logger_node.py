#!/usr/bin/env python3
"""
dt_logger_node.py  —  Digital Entity backend (Option B)
=======================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

The Digital Entity (DE) has two halves in this PoC:
  - the WEB DASHBOARD (the live visual mirror the user looks at), and
  - this node, the DE's SERVER-SIDE aggregator / historian.

It subscribes to the Physical Entity's state stream (/pe/state, published by
pe_state_node) and to farm events (/farm_action), keeps a timestamped log, and
republishes a single consolidated /dt/status (every 5 s and on every event).
That /dt/status is what proves Goal ② state-synchronisation: it carries the
PE's internal state (battery, sensor health, motor, mode, localization quality)
mirrored on the DE side, plus a link_age_s freshness figure showing the mirror
is near-real-time.

Topics:
  SUB /pe/state    — PE internal state (JSON) from pe_state_node
  SUB /farm_action — zone spray/fertilize events from zone_monitor_node
  PUB /dt/status   — consolidated DE view (every 5 s + on each event)

Services:
  /get_dt_status — current summary (counts + mirrored PE state + link freshness)
  /get_dt_log    — full action history
"""
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DTLoggerNode(Node):

    def __init__(self):
        super().__init__('dt_logger_node')
        self._log         = []
        self._spray_count = 0
        self._fert_count  = 0
        self._zones       = set()

        self._pe_state    = None     # latest mirrored PE state (dict)
        self._pe_state_t  = None     # ROS time (sec) when it last arrived

        self.declare_parameter('pe_state_topic', '/pe/state')
        self.create_subscription(String, self.get_parameter('pe_state_topic').value,
                                 self._pe_state_cb, 10)
        self.create_subscription(String, '/farm_action', self._action_cb, 10)

        self._status_pub = self.create_publisher(String, '/dt/status', 10)
        self.create_service(Trigger, '/get_dt_status', self._status_srv)
        self.create_service(Trigger, '/get_dt_log',    self._log_srv)
        self.create_timer(5.0, self._broadcast)

        self.get_logger().info('DT Logger (Digital Entity backend) started')
        self.get_logger().info('Query: ros2 service call /get_dt_status std_srvs/srv/Trigger')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pe_state_cb(self, msg: String):
        try:
            self._pe_state = json.loads(msg.data)
            self._pe_state_t = self._now()
        except Exception:
            return

    def _action_cb(self, msg: String):
        try:
            event = json.loads(msg.data)
        except Exception:
            return
        event['logged_at'] = datetime.now().isoformat()
        self._log.append(event)
        self._zones.add(event.get('zone', '?'))
        if event.get('action') == 'spray':
            self._spray_count += 1
        elif event.get('action') == 'fertilize':
            self._fert_count += 1
        self.get_logger().info(
            f'[DE LOG] {event.get("action", "?").upper()} | zone={event.get("zone")} | '
            f'spray={self._spray_count}  fertilize={self._fert_count}')
        self._broadcast()

    def _broadcast(self):
        self._status_pub.publish(String(data=json.dumps(self._build())))

    def _build(self):
        link_age = (round(self._now() - self._pe_state_t, 2)
                    if self._pe_state_t is not None else None)
        return {
            'pe_state':          self._pe_state,            # mirrored PE internal state
            'link_age_s':        link_age,                  # freshness of the PE->DE mirror
            'twin_synced':       (link_age is not None and link_age < 2.0),
            'spray_actions':     self._spray_count,
            'fertilize_actions': self._fert_count,
            'total_actions':     len(self._log),
            'zones_covered':     sorted(self._zones),
            'last_action':       self._log[-1] if self._log else None,
        }

    def _status_srv(self, req, res):
        res.success = True
        res.message = json.dumps(self._build(), indent=2)
        return res

    def _log_srv(self, req, res):
        res.success = True
        res.message = json.dumps(self._log, indent=2)
        return res


def main():
    rclpy.init()
    node = DTLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
