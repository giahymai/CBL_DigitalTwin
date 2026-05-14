#!/usr/bin/env python3
"""
gazebo_twin.launch.py — Gazebo Digital Twin
============================================
Based on course tb3_scanner_dt/gazebo_twin.launch.py.

Uses PushRosNamespace('sim') to namespace all Gazebo topics:
  /sim/scan      ← Gazebo LiDAR (separated from real /scan)
  /sim/cmd_vel   → Gazebo robot (receives twin commands)
  /sim/odom      ← Gazebo odometry

Usage: ros2 launch farm_twin_poc gazebo_twin.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        )
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/sim/robot_description',
            '-name',  'turtlebot3',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        PushRosNamespace('sim'),
        gazebo,
        spawn_robot,
    ])
