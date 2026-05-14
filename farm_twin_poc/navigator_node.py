#!/usr/bin/env python3
"""
navigator_node.py  —  Node 5: Autonomous Navigator
===================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

Sends Nav2 NavigateToPose goals to drive the robot autonomously
to each farm zone in sequence.

Topics:
  SUB /odom              — current position
  PUB /navigator/status  — navigation progress

Services:
  /start_navigation — begin navigating all zones in sequence
  /stop_navigation  — cancel current navigation
  /nav_status       — query navigation state

IMPORTANT: Nav2 must be running before calling /start_navigation.
           Set "2D Pose Estimate" in RViz after Nav2 starts.

ADJUST WAYPOINTS to match FARM_ZONES in zone_monitor_node.py.
"""
import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

WAYPOINTS = [
    {'name': 'spray_zone_A',     'x':  1.0, 'y':  0.0, 'action': 'spray'},
    {'name': 'fertilize_zone_B', 'x':  0.0, 'y':  1.0, 'action': 'fertilize'},
    {'name': 'spray_zone_C',     'x': -1.0, 'y':  0.0, 'action': 'spray'},
    {'name': 'fertilize_zone_D', 'x':  0.0, 'y': -1.0, 'action': 'fertilize'},
]


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator_node')
        self._x = self._y = 0.0
        self._navigating: bool = False
        self._current: Optional[str] = None
        self._completed: list = []

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._status_pub = self.create_publisher(String, '/navigator/status', 10)
        self.create_service(Trigger, '/start_navigation', self._start_srv)
        self.create_service(Trigger, '/stop_navigation',  self._stop_srv)
        self.create_service(Trigger, '/nav_status',       self._nav_status_srv)
        self.create_timer(3.0, self._broadcast)

        self.get_logger().info(f'Navigator Node started | {len(WAYPOINTS)} waypoints')
        for wp in WAYPOINTS:
            self.get_logger().info(f'  {wp["action"].upper():<11} {wp["name"]} at ({wp["x"]}, {wp["y"]})')
        self.get_logger().info(
            'Start: ros2 service call /start_navigation std_srvs/srv/Trigger'
        )
        self.get_logger().info('IMPORTANT: Nav2 must be running and pose estimate set first')

    def _odom_cb(self, msg):
        self._x, self._y = msg.pose.pose.position.x, msg.pose.pose.position.y

    def _broadcast(self):
        msg = String()
        msg.data = (f'navigating={self._navigating} | current={self._current} | '
                    f'completed={self._completed}')
        self._status_pub.publish(msg)

    def _start_srv(self, req, res):
        if self._navigating:
            res.success = False; res.message = 'Already navigating'; return res
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            res.success = False
            res.message = 'Nav2 not available. Run: ros2 launch farm_twin_poc navigation.launch.py'
            return res
        self._navigating = True
        self._completed = []
        threading.Thread(target=self._run, daemon=True).start()
        res.success = True; res.message = f'Navigation started — {len(WAYPOINTS)} waypoints'; return res

    def _stop_srv(self, req, res):
        self._navigating = False; self._current = None
        res.success = True; res.message = 'Navigation stopped'; return res

    def _nav_status_srv(self, req, res):
        remaining = [w['name'] for w in WAYPOINTS if w['name'] not in self._completed]
        res.success = True
        res.message = (f'navigating={self._navigating}\ncurrent={self._current}\n'
                       f'completed={self._completed}\nremaining={remaining}')
        return res

    def _run(self):
        for wp in WAYPOINTS:
            if not self._navigating:
                break
            self._current = wp['name']
            self.get_logger().info(f'Navigating to {wp["name"]} at ({wp["x"]}, {wp["y"]})')
            if self._go_to(wp['x'], wp['y']):
                self._completed.append(wp['name'])
                self.get_logger().info(f'Reached {wp["name"]}')
                time.sleep(1.0)
            else:
                self.get_logger().warn(f'Failed to reach {wp["name"]} — skipping')
        self._navigating = False
        self._current = None
        self.get_logger().info(f'Navigation done — {len(self._completed)}/{len(WAYPOINTS)} zones visited')

    def _go_to(self, x: float, y: float) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.result() or not future.result().accepted:
            return False
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=60.0)
        if not result.result():
            return False
        from action_msgs.msg import GoalStatus
        return result.result().status == GoalStatus.STATUS_SUCCEEDED


def main():
    rclpy.init()
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
