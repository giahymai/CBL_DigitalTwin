#!/usr/bin/env python3
"""
farm_twin.launch.py — Farm Twin, Option B (no physical robot)
=============================================================
Farm Twin PoC | Team 5 Terra Minds | Course 2IRR10

ONE command brings up the whole digital-twin system:

  PHYSICAL ENTITY (PE) — the source of truth
    - Gazebo TurtleBot3 in the farm world (worlds/lab_world.sdf)
    - Nav2 + AMCL (planned navigation + obstacle avoidance)
    - navigator_node   : drives the farm-zone tour + return-home
    - zone_monitor_node: fires /farm_action at each zone
    - pe_state_node    : internal state (battery, sensor health, motor, mode),
                         the /cmd_vel command gate, and LiDAR fault injection

  DIGITAL ENTITY (DE) — the second representation that mirrors the PE
    - rosbridge_websocket : bridges ROS <-> the browser
    - a static web server : serves web/index.html (the live dashboard)
    - dt_logger_node      : DE-side aggregator/historian -> /dt/status

USAGE (Gazebo, at home / in the container):
  ros2 launch farm_twin_poc farm_twin.launch.py
  # GPU-less machine (Docker-in-WSL): add headless:=true
  # then open http://localhost:8080 in a browser for the dashboard.

ARGS:
  headless        true = Gazebo server only, no 3D window (GPU-less machines)
  dashboard       true (default) = start rosbridge + web server
  dashboard_port  http port for the web dashboard (default 8080)
  rosbridge_port  websocket port roslibjs connects to   (default 9090)
  map / params_file / world / x_pose / y_pose / zone_signal
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import (PythonLaunchDescriptionSource,
                                               AnyLaunchDescriptionSource)
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2       = get_package_share_directory('turtlebot3_navigation2')
    pkg_share      = get_package_share_directory('farm_twin_poc')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    map_yaml       = LaunchConfiguration('map')
    world          = LaunchConfiguration('world')
    x_pose         = LaunchConfiguration('x_pose')
    y_pose         = LaunchConfiguration('y_pose')
    headless       = LaunchConfiguration('headless')
    nav2_params    = LaunchConfiguration('params_file')
    zone_signal    = LaunchConfiguration('zone_signal')
    dashboard      = LaunchConfiguration('dashboard')
    dashboard_port = LaunchConfiguration('dashboard_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    default_world  = os.path.join(pkg_share, 'worlds', 'lab_world.sdf')
    default_map    = os.path.join(pkg_share, 'maps', 'lab_map.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_sim.yaml')
    web_dir        = os.path.join(pkg_share, 'web')

    # headless:=true -> run Gazebo as server only (-s), no 3D GUI. On a GPU-less
    # box the ogre2 GUI software-renders and starves the sim clock; headless
    # removes that load. You still get the web dashboard (and RViz from Nav2).
    gz_flags = PythonExpression(["'-s -r ' if '", headless, "' == 'true' else '-r '"])

    launch_file_dir = os.path.join(pkg_tb3_gazebo, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('x_pose', default_value='3'),
        DeclareLaunchArgument('y_pose', default_value='3'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = Gazebo server only (GPU-less)'),
        DeclareLaunchArgument('zone_signal', default_value='pause',
                              description="'pause' (default) or 'spin' at each zone"),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='start rosbridge + the web dashboard'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),

        # ============================ PHYSICAL ENTITY ========================

        # 1) Gazebo with the farm world (default namespace -> clean TF tree).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ),
            launch_arguments={'gz_args': [gz_flags, world]}.items(),
        ),

        # 2) Spawn TurtleBot3 (its bridge publishes /clock /tf /odom /scan /cmd_vel).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),

        # 2b) robot_state_publisher: static part of the TF tree from the URDF.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command([
                    'xacro ',
                    os.path.join(pkg_tb3_gazebo, 'urdf', 'turtlebot3_burger.urdf'),
                ]),
            }],
        ),

        # 3) Nav2 + AMCL with the sim-tuned params (scan = /scan_filtered).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map':          map_yaml,
                'params_file':  nav2_params,
            }.items(),
        ),

        # 4) PE state core: internal state + /cmd_vel gate + LiDAR fault.
        Node(
            package='farm_twin_poc',
            executable='pe_state_node',
            name='pe_state_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 5) Navigator (Nav2 Simple Commander). Its command + dwell/spin go
        # through the PE gate (/cmd_vel_pe), not straight to /cmd_vel.
        Node(
            package='farm_twin_poc',
            executable='nav2_navigator',
            name='navigator_node',
            output='screen',
            parameters=[{
                'odom_topic':       '/odom',
                'battery_topic':    '/battery_state',
                'set_initial_pose': True,
                'home_x':           3.0,
                'home_y':           3.0,
                'home_yaw':         0.0,
                'zone_signal':      zone_signal,
                'spin_cmd_topic':   '/cmd_vel_pe',
                'use_sim_time':     use_sim_time,
            }],
        ),

        # 6) Zone monitor — detect zones in the MAP frame via TF.
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

        # ============================ DIGITAL ENTITY ========================

        # 7) DE backend: aggregate /pe/state + /farm_action -> /dt/status.
        Node(
            package='farm_twin_poc',
            executable='dt_logger_node',
            name='dt_logger_node',
            output='screen',
            parameters=[{
                'pe_state_topic': '/pe/state',
                'use_sim_time':   use_sim_time,
            }],
        ),

        # 8) rosbridge_websocket — lets the browser dashboard talk to ROS.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rosbridge_server'),
                             'launch', 'rosbridge_websocket_launch.xml')
            ),
            launch_arguments={'port': rosbridge_port}.items(),
            condition=IfCondition(dashboard),
        ),

        # 9) Static web server for the dashboard page (web/index.html).
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server',
                 dashboard_port, '--directory', web_dir],
            output='screen',
            condition=IfCondition(dashboard),
        ),
    ])
