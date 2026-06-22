#!/usr/bin/env python3
"""
physical_entity.launch.py — the simulated robot ("PE") side
============================================================
Farm Twin PoC | Team 5 Terra Minds

Brings up everything that represents the physical robot:

  - Gazebo running worlds/new_world.world
  - The TurtleBot3 Burger spawned at (x_pose, y_pose)
  - robot_state_publisher
  - Nav2 + AMCL using the baked maps/new_world_map.yaml
  - navigator_node (onboard "drive to a destination" worker)
  - battery_sim_node (fake /battery_state — Gazebo doesn't model this)

Anything that observes, orchestrates, or talks to a human lives on the
Digital Entity side — see digital_entity.launch.py.

USAGE:
  colcon build --packages-select farm_twin_poc && source install/setup.bash
  # terminal A:
  ros2 launch farm_twin_poc physical_entity.launch.py
  # wait until you see "Nav2 is active. Awaiting /destination messages."
  # terminal B:
  ros2 launch farm_twin_poc digital_entity.launch.py
  # terminal C (or click the button in the browser):
  ros2 service call /start_navigation std_srvs/srv/Trigger

ARGS:
  map           occupancy-grid yaml          (default: pkg maps/new_world_map.yaml)
  world         Gazebo world sdf             (default: pkg worlds/new_world.world)
  x_pose,y_pose spawn position               (default: 1.5, -2.0)
  headless      true = Gazebo server only    (default: false)
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
    pkg_share      = get_package_share_directory('farm_twin_poc')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    world        = LaunchConfiguration('world')
    x_pose       = LaunchConfiguration('x_pose')
    y_pose       = LaunchConfiguration('y_pose')
    headless     = LaunchConfiguration('headless')
    nav2_params  = LaunchConfiguration('params_file')

    # headless:=true runs Gazebo as server only (-s). On a GPU-less machine
    # (Docker-in-WSL, etc.) the ogre2 GUI software-renders and starves the
    # sim clock, killing Nav2's controller. Headless removes that load.
    gz_flags = PythonExpression(
        ["'-s -r ' if '", headless, "' == 'true' else '-r '"])

    default_world  = os.path.join(pkg_share, 'worlds',  'new_world.world')
    default_map    = os.path.join(pkg_share, 'maps',    'new_world_map.yaml')
    default_params = os.path.join(pkg_share, 'config',  'nav2_sim.yaml')
    launch_file_dir = os.path.join(pkg_tb3_gazebo, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map',          default_value=default_map),
        DeclareLaunchArgument('params_file',  default_value=default_params),
        DeclareLaunchArgument('world',        default_value=default_world),
        DeclareLaunchArgument('x_pose',       default_value='1.5'),
        DeclareLaunchArgument('y_pose',       default_value='-2.0'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = Gazebo server only (no GUI)'),

        # Gazebo with the world.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ),
            launch_arguments={'gz_args': [gz_flags, world]}.items(),
        ),

        # Spawn TurtleBot3 in the default namespace (clean TF tree).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),

        # robot_state_publisher
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

        # 3) Nav2 + AMCL with the baked map.
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

        # 4) Navigator — per-destination Nav2 driver. Subscribes to
        # /destination and drives there. /start_navigation lives on the
        # mission_dispatcher in digital_entity.launch.py.
        Node(
            package='farm_twin_poc',
            executable='nav2_navigator',
            name='navigator_node',
            output='screen',
            parameters=[{
                'odom_topic':       '/odom',
                'battery_topic':    '/battery_state',
                # MUST be True. nav2_simple_commander's waitUntilNav2Active()
                # calls _waitForInitialPose() which loops publishing
                # BasicNavigator.initial_pose on /initialpose until AMCL
                # responds. Default-constructed (0,0,0) would silently
                # overwrite the AMCL seed.
                'set_initial_pose': True,
                'home_x':           1.5,
                'home_y':          -2.0,
                'home_yaw':         0.0,
                'use_sim_time':     use_sim_time,
            }],
        ),

        # 5) Battery simulator — drains a fake /battery_state. The
        # navigator's return-home threshold reads this.
        Node(
            package='farm_twin_poc',
            executable='battery_sim_node',
            name='battery_sim_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
