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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    my_pkg_share     = get_package_share_directory('farm_twin_poc')

    # path to the 3d model world file
    world = os.path.join(my_pkg_share, 'worlds', 'lab_world.sdf')

    #gazebo = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        PathJoinSubstitution([
    #            FindPackageShare('turtlebot3_gazebo'),
    #            'launch',
    #            'turtlebot3_world.launch.py'
    #        ])
    #    )
    #)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items()
    )


    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    return LaunchDescription([
        # At home: keep defaults (3, 3) — middle of the Gazebo room.
        # At lab: pass the robot's real starting map-frame coords so the twin
        # spawns at the same location as the physical robot.
        #   ros2 launch farm_twin_poc gazebo_twin.launch.py x_pose:=<home_x> y_pose:=<home_y>
        DeclareLaunchArgument('x_pose', default_value='3',
                              description='Gazebo twin spawn x (map-frame). At lab: use home_x from amcl_pose.'),
        DeclareLaunchArgument('y_pose', default_value='3',
                              description='Gazebo twin spawn y (map-frame). At lab: use home_y from amcl_pose.'),
        PushRosNamespace('sim'),
        gazebo,
        spawn_robot,
    ])
