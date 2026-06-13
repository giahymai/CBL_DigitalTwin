#!/usr/bin/env python3
"""
battery_sim_node.py — fake sensor_msgs/BatteryState publisher
==============================================================
Farm Twin PoC | Team 5 Terra Minds

Gazebo doesn't model the TurtleBot3 Burger's battery, but the navigator
already listens on /battery_state to trigger return-home below a
threshold. This node publishes a plausible BatteryState so the rest of
the stack (navigator, web UI) behaves as it would on the real robot.

Behaviour
---------
- Starts at `initial_percent` (default 100 %).
- Drains at `drain_per_hour_idle` while the robot is stationary,
  `drain_per_hour_moving` while /odom shows motion above
  `move_threshold` (m/s on linear.x, or > 0.1 rad/s yaw).
- Voltage is a linear interpolation between 9.5 V (empty) and 12.6 V
  (full) — the LiPo range a TB3 Burger sees.
- Publishes at `publish_rate_hz` Hz (default 1).

Topic
  PUB <battery_topic>   sensor_msgs/BatteryState (default /battery_state)

Subscriber
  SUB <odom_topic>      nav_msgs/Odometry        (default /odom)
"""
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64


class BatterySimNode(Node):

    def __init__(self):
        super().__init__('battery_sim_node')

        self.declare_parameter('battery_topic',         '/battery_state')
        self.declare_parameter('odom_topic',            '/odom')
        self.declare_parameter('initial_percent',       100.0)
        # Realistic-ish defaults: TB3 Burger LiPo lasts ~2.5 h driving,
        # idle is negligible drain. Bump these (e.g. 60 / 120) for a
        # short-demo run where you want to see the return-home trigger.
        self.declare_parameter('drain_per_hour_idle',    5.0)
        self.declare_parameter('drain_per_hour_moving', 20.0)
        self.declare_parameter('publish_rate_hz',        1.0)
        self.declare_parameter('move_threshold',        0.02)   # m/s

        gp = self.get_parameter
        self._percent      = float(gp('initial_percent').value)
        # Stored as percentage points per second so each tick can just
        # multiply by dt without per-tick unit conversion.
        self._drain_idle   = float(gp('drain_per_hour_idle').value) / 3600.0
        self._drain_moving = float(gp('drain_per_hour_moving').value) / 3600.0
        self._move_thresh  = float(gp('move_threshold').value)

        self._moving       = False
        self._last_t       = time.monotonic()

        self.create_subscription(
            Odometry, str(gp('odom_topic').value), self._odom_cb, 10)
        # Manual override from the web UI — POST /api/set_battery on the
        # web server publishes a Float64 (0..100) here, and the next
        # _tick will report the new value.
        self.create_subscription(
            Float64, '/battery_set_level', self._cb_set_level, 10)
        self._pub = self.create_publisher(
            BatteryState, str(gp('battery_topic').value), 10)
        self.create_timer(
            1.0 / max(0.1, float(gp('publish_rate_hz').value)), self._tick)

        self.get_logger().info(
            f'battery_sim_node up — {self._percent:.0f}% start, '
            f'drain idle {gp("drain_per_hour_idle").value:.1f}%/h, '
            f'moving {gp("drain_per_hour_moving").value:.1f}%/h')

    def _cb_set_level(self, msg: Float64) -> None:
        # Clamp so a bad UI value can never wedge the sim into a NaN
        # or negative state — both would crash downstream subscribers.
        val = max(0.0, min(100.0, float(msg.data)))
        self.get_logger().info(
            f'[BATTERY] manual override: {self._percent:.1f}% -> {val:.1f}%')
        self._percent = val

    def _odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist
        # Either pushing forward OR spinning in place counts as moving.
        # The 360° spray/fertilize spin would otherwise look "idle" to a
        # linear-only check and under-drain the battery.
        self._moving = (abs(v.linear.x)  > self._move_thresh
                        or abs(v.angular.z) > 0.1)

    def _tick(self) -> None:
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now
        rate = self._drain_moving if self._moving else self._drain_idle
        self._percent = max(0.0, self._percent - rate * dt)

        msg = BatteryState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # ROS convention: percentage is 0..1, voltage in volts, charge in Ah,
        # current in A (negative when discharging).
        msg.percentage      = self._percent / 100.0
        msg.voltage         = 9.5 + 3.1 * (self._percent / 100.0)
        msg.current         = -0.5 if self._moving else -0.1
        msg.charge          = 1.8 * (self._percent / 100.0)
        msg.capacity        = 1.8
        msg.design_capacity = 1.8
        msg.power_supply_status     = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health     = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = BatterySimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
