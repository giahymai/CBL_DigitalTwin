#!/usr/bin/env python3
"""
digital_entity.launch.py — the orchestration / DE side
==============================================================
Farm Twin PoC | Team 5 Terra Minds

Brings up everything that observes, plans, logs, or talks to a human:

  - mission_dispatcher_node — owns WAYPOINTS, exposes /start_navigation,
                              walks the list by publishing /destination
                              and waiting for /destination_reached
  - zone_monitor_node       — detects zone entry via TF, publishes
                              /farm_action
  - dt_logger_node          — aggregates /farm_action into a history
                              and publishes /dt/status
  - web_server_node         — bridges the ROS state to an HTTP+SSE
                              endpoint and serves the browser UI

The robot itself, Nav2, and the onboard navigator + battery sim live on
the Physical Entity side — see physical_entity.launch.py.

USAGE:
  colcon build --packages-select farm_twin_poc && source install/setup.bash
  # in another terminal, start the PE first:
  ros2 launch farm_twin_poc physical_entity.launch.py
  # then, once Nav2 is active:
  ros2 launch farm_twin_poc digital_entity.launch.py
  # open http://localhost:8080 and click Start Navigation
  # (or: ros2 service call /start_navigation std_srvs/srv/Trigger)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # The PE runs Gazebo, so use_sim_time defaults to true here too
        # — both sides need to agree on the clock source. Override with
        # use_sim_time:=false if you ever wire this DE to a real robot.
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 1) Mission dispatcher — owns the WAYPOINTS tour. Exposes
        # /start_navigation (the entry point the user / browser button
        # calls) and walks the list one /destination at a time,
        # advancing on each /destination_reached ack from the navigator.
        Node(
            package='farm_twin_poc',
            executable='mission_dispatcher_node',
            name='mission_dispatcher_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),

        # 2) Zone monitor — fires /farm_action when the robot's
        # map-frame position enters a zone radius. Uses TF because /odom
        # drifts and AMCL corrects it; reading the corrected pose via
        # TF keeps zone entry aligned with the map.
        Node(
            package='farm_twin_poc',
            executable='zone_monitor_node',
            name='zone_monitor_node',
            output='screen',
            parameters=[{
                'position_source': 'tf',
                'global_frame':    'map',
                'robot_frame':     'base_link',
                'use_sim_time':    use_sim_time,
            }],
        ),

        # 3) DT logger — aggregates /farm_action into a timestamped log,
        # publishes /dt/status with counters and recent_actions for the
        # web UI.
        Node(
            package='farm_twin_poc',
            executable='dt_logger_node',
            name='dt_logger_node',
            output='screen',
            parameters=[{
                'real_odom_topic': '/odom',
                'sim_odom_topic':  '/odom',
                'use_sim_time':    use_sim_time,
            }],
        ),

        # 4) Web server — subscribes to every relevant topic, serves
        # web/index.html + an SSE stream at http://<host>:8080. The
        # Start Navigation button in the UI POSTs back here, which
        # forwards to /start_navigation on the dispatcher above.
        Node(
            package='farm_twin_poc',
            executable='web_server_node',
            name='web_server_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
