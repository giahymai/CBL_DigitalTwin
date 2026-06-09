#!/usr/bin/env python3
"""
gazebo_nav2_demo.launch.py — Nav2 demo in the LAB world (NO namespace)
======================================================================
Farm Twin PoC | Team 5 Terra Minds

Standalone Nav2 demo for "robot drives a planned path to each farm zone",
running the SAME lab world used by the twin (worlds/lab_world.sdf) but WITHOUT
the `sim` namespace, so the TF tree is clean (map -> odom -> base_link) and
Nav2 works.

Why the lab world + lab map?  Your lab map (.pgm/.yaml) was produced by SLAM
and converted into worlds/lab_world.sdf via scripts/map_to_world.py, so the
Gazebo walls and the Nav2 map MATCH. (Never mix turtlebot3_world with the lab
map — the walls wouldn't line up and AMCL would fail.)

Why no namespace?  The twin launch uses PushRosNamespace('sim'), but Nav2 does
NOT prefix TF frames with the namespace, so it would look for `odom`/
`base_link` that don't exist under `sim` -> "Invalid frame ID 'odom'". This
demo therefore runs everything in the default namespace.

USAGE (Gazebo, at home):
  colcon build --packages-select farm_twin_poc && source install/setup.bash
  ros2 launch farm_twin_poc gazebo_nav2_demo.launch.py map:=$HOME/map.yaml
  # In RViz: "2D Pose Estimate" at the robot spawn, wait for "Nav2 is active":
  ros2 service call /start_navigation std_srvs/srv/Trigger

ARGS:
  map           path to your SLAM lab map yaml   (default: ~/map.yaml)
  world         path to the lab world sdf        (default: pkg worlds/lab_world.sdf)
  x_pose,y_pose robot spawn position             (default: 3, 3 — match twin)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2       = get_package_share_directory('turtlebot3_navigation2')
    pkg_ros_gz     = get_package_share_directory('ros_gz_sim')
    pkg_share      = get_package_share_directory('farm_twin_poc')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    world        = LaunchConfiguration('world')
    x_pose       = LaunchConfiguration('x_pose')
    y_pose       = LaunchConfiguration('y_pose')
    headless     = LaunchConfiguration('headless')
    nav2_params  = LaunchConfiguration('params_file')

    # headless:=true runs Gazebo as server only (-s), no 3D GUI window. On a
    # machine WITHOUT a real GPU (e.g. Docker-in-WSL, where /dev/dri is absent)
    # the ogre2 GUI software-renders and starves the sim, which makes /clock
    # stutter and jump backwards ("Detected jump back in time"), killing Nav2's
    # controller. Headless removes that load so Nav2 can run. You lose the 3D
    # view, but RViz still shows the map/robot. set_initial_pose seeds AMCL so
    # no manual "2D Pose Estimate" click is required.
    gz_flags = PythonExpression(
        ["'-s -r ' if '", headless, "' == 'true' else '-r '"])

    default_world = os.path.join(pkg_share, 'worlds', 'lab_world.sdf')
    # Map bundled with the package, generated from lab_world.sdf via
    # scripts/world_to_map.py. Matches the world 1:1 so the at-home demo runs
    # with no manual SLAM. Override with map:=... to use your own.
    default_map = os.path.join(pkg_share, 'maps', 'lab_map.yaml')
    # Nav2 params tuned for a slow (GPU-less) simulator: the LiDAR renders late,
    # so /scan and TF lag. This copy of turtlebot3's burger.yaml relaxes the
    # collision_monitor scan source_timeout (0.2 -> 5.0 s) and TF tolerances so
    # Nav2 doesn't keep stopping the robot for "invalid/stale source". The lab
    # (navigation.launch.py) keeps the stock params — real robot, real time.
    default_params = os.path.join(pkg_share, 'config', 'nav2_sim.yaml')

    launch_file_dir = os.path.join(pkg_tb3_gazebo, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('world', default_value=default_world),
        # Spawn at (3,3): top-left area of the Gazebo room, consistent with
        # gazebo_twin.launch.py. zone_monitor uses position_source:=tf (map frame
        # via AMCL), so zone detection is independent of the odom origin.
        # AMCL is seeded at (3,3) via nav2_sim.yaml initial_pose so odom+map align.
        DeclareLaunchArgument('x_pose', default_value='3'),
        DeclareLaunchArgument('y_pose', default_value='3'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = Gazebo server only (no GUI); use on GPU-less '
                        'machines to avoid sim-clock jumps'),

        # 1) Gazebo with the LAB world (no namespace).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ),
            launch_arguments={'gz_args': [gz_flags, world]}.items(),
        ),

        # 2) Spawn TurtleBot3 in the default namespace (clean TF tree).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),

        # 2b) NOTE: do NOT add a separate /tf + /clock bridge here.
        # spawn_turtlebot3.launch.py (included above) already starts the TB3
        # parameter_bridge from turtlebot3_burger_bridge.yaml, which bridges
        # /clock, /tf, /odom, /scan, /cmd_vel, /imu and /joint_states. Adding a
        # second bridge for /clock and /tf makes BOTH republish the same gz
        # topics out of order -> "Detected jump back in time. Clearing TF
        # buffer" floods, AMCL/costmap TF breaks and Nav2's controller freezes
        # the robot. One bridge only.

        # 2c) robot_state_publisher: provides the STATIC part of the tree
        # (base_footprint -> base_link -> wheels/scan) from the TB3 URDF.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command([
                    'xacro ',
                    os.path.join(
                        get_package_share_directory('turtlebot3_gazebo'),
                        'urdf', 'turtlebot3_burger.urdf'),
                ]),
            }],
        ),

        # 3) Nav2 + AMCL localization with the LAB map.
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

        # 4) Navigator (Nav2 Simple Commander).
        Node(
            package='farm_twin_poc',
            executable='nav2_navigator',
            name='navigator_node',
            output='screen',
            parameters=[{
                'odom_topic':       '/odom',
                'battery_topic':    '/battery_state',
                # AMCL self-seeds at (0,0) via nav2_sim.yaml (set_initial_pose),
                # so the navigator does not need to re-seed (that retry loop
                # spammed "Setting initial pose" + transform-timeout warnings).
                'set_initial_pose': False,
                'home_x':           3.0,     # = spawn; return_home comes back here
                'home_y':           3.0,
                'home_yaw':         0.0,
                'use_sim_time':     use_sim_time,
            }],
        ),

        # 5) Zone monitor + DT logger (so /farm_action fires at zones).
        Node(
            package='farm_twin_poc',
            executable='zone_monitor_node',
            name='zone_monitor_node',
            output='screen',
            # Nav2: detect zones in the MAP frame via TF. /odom drifts and AMCL
            # corrects it, so the robot reaches a tile in map coords but /odom
            # reports a different number -> zone detection on /odom never fires.
            parameters=[{
                'position_source': 'tf',
                'global_frame':    'map',
                'robot_frame':     'base_link',
                'use_sim_time':    use_sim_time,
            }],
        ),
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
    ])
