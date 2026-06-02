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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    default_world = os.path.join(pkg_share, 'worlds', 'lab_world.sdf')
    # Map bundled with the package, generated from lab_world.sdf via
    # scripts/world_to_map.py. Matches the world 1:1 so the at-home demo runs
    # with no manual SLAM. Override with map:=... to use your own.
    default_map = os.path.join(pkg_share, 'maps', 'lab_map.yaml')

    launch_file_dir = os.path.join(pkg_tb3_gazebo, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('world', default_value=default_world),
        # Spawn at the WORLD ORIGIN (0,0). This makes odom == map == world: the
        # robot's /odom (origin = spawn) coincides with the map frame, so the
        # FARM_ZONES coords in zone_monitor_node (which reads /odom) line up with
        # the Gazebo zone markers and the Nav2 goals. Mirrors the lab convention
        # "zones are metres from robot start". Spawning elsewhere (e.g. 3,3)
        # offsets /odom from the map and zone_monitor never fires /farm_action.
        DeclareLaunchArgument('x_pose', default_value='0'),
        DeclareLaunchArgument('y_pose', default_value='0'),

        # 1) Gazebo with the LAB world (no namespace).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ),
            launch_arguments={'gz_args': ['-r ', world]}.items(),
        ),

        # 2) Spawn TurtleBot3 in the default namespace (clean TF tree).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),

        # 2b) Bridge the gz `tf` topic to ROS /tf. On gz_sim (Harmonic) the
        # DiffDrive plugin publishes odom->base_footprint on a gz topic named
        # `tf`; without this bridge the ROS TF tree is EMPTY (the exact cause
        # of "Invalid frame ID 'odom'"). Also bridge /clock for sim time.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_tf_bridge',
            output='screen',
            arguments=[
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

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
                'set_initial_pose': True,    # seed AMCL at home pose below
                'home_x':           0.0,     # = spawn; return_home comes back here
                'home_y':           0.0,
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
            parameters=[{'odom_topic': '/odom', 'use_sim_time': use_sim_time}],
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
