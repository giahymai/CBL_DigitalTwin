import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch Nav2 + autonomous Farm Twin navigation on the REAL robot (lab).
    Requires a map.yaml produced by SLAM (see README B3).

    This brings up, in ONE command:
      - Nav2 (turtlebot3_navigation2) with your lab map + AMCL
      - nav2_navigator   (drives zones in sequence, auto return-home on low battery)
      - zone_monitor_node (fires /farm_action at each zone)
      - dt_logger_node    (Digital Entity log + /dt/status)

    Robot bringup (ros2 launch turtlebot3_bringup robot.launch.py) must already
    be running over SSH, and ROS_DOMAIN_ID must match the robot.

    AMCL auto-seeds at home_x/home_y (default 3.0, 3.0) — place the robot at
    that position and RViz2 will show the correct location immediately, no
    "2D Pose Estimate" click required. If the robot is placed elsewhere, pass
    home_x:=X home_y:=Y on the command line; the AMCL seed follows home_x/y.

    The bundled maps/lab_map.yaml (real SLAM scan of the lab) is used by default —
    no map copy or ~/map.yaml needed. Override with map:=<path> only if you have
    a newer SLAM scan.

    USAGE (real robot, lab):
      ros2 launch farm_twin_poc navigation.launch.py

    Wait for "Nav2 is active", then:
      ros2 service call /start_navigation std_srvs/srv/Trigger

    Override start/home position if robot is not at (3, 3):
      ros2 launch farm_twin_poc navigation.launch.py home_x:=X home_y:=Y

    ARGS:
      map                     path to map yaml  (default: bundled maps/lab_map.yaml)
      params_file             Nav2 params yaml  (default: nav2_lab.yaml)
      use_sim_time            false on real robot (default: false)
      home_x, home_y, home_yaw  robot start pose + AMCL seed (default: 3.0, 3.0, 0.0)
      return_battery_percent  low-battery return threshold % (default: 20)
      set_initial_pose        seed AMCL via Nav2 API (default: true)
    """
    pkg_nav2  = get_package_share_directory('turtlebot3_navigation2')
    pkg_share = get_package_share_directory('farm_twin_poc')

    use_sim_time  = LaunchConfiguration('use_sim_time')
    map_yaml      = LaunchConfiguration('map')
    home_x        = LaunchConfiguration('home_x')
    home_y        = LaunchConfiguration('home_y')
    home_yaw      = LaunchConfiguration('home_yaw')
    ret_batt      = LaunchConfiguration('return_battery_percent')
    set_pose      = LaunchConfiguration('set_initial_pose')
    nav2_params   = LaunchConfiguration('params_file')

    default_map    = os.path.join(pkg_share, 'maps', 'lab_map.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_lab.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Default start position matches the fixed robot placement and Gazebo spawn.
        # AMCL seeds at this position so RViz2 shows the correct location on startup.
        DeclareLaunchArgument('home_x', default_value='3.0'),
        DeclareLaunchArgument('home_y', default_value='3.0'),
        DeclareLaunchArgument('home_yaw', default_value='0.0'),
        DeclareLaunchArgument('return_battery_percent', default_value='20.0'),
        DeclareLaunchArgument('set_initial_pose', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 params yaml (default: nav2_lab.yaml)'),

        # 1) Nav2 + AMCL with lab-tuned params.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'map':          map_yaml,
                'use_sim_time': use_sim_time,
                'params_file':  nav2_params,
            }.items(),
        ),

        # 2) Nav2 navigator (zones in sequence + return-home).
        Node(
            package='farm_twin_poc',
            executable='nav2_navigator',
            name='navigator_node',
            output='screen',
            parameters=[{
                'odom_topic':             '/odom',
                'battery_topic':          '/battery_state',
                'return_battery_percent': ret_batt,
                'home_x':                 home_x,
                'home_y':                 home_y,
                'home_yaw':               home_yaw,
                'set_initial_pose':       set_pose,
                'use_sim_time':           use_sim_time,
            }],
        ),

        # 3) Zone monitor — fires /farm_action at each zone. Detect in the MAP
        # frame via TF: under Nav2 /odom drifts and AMCL corrects it, so the
        # robot reaches a zone in map coords while /odom reads something else.
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

        # 4) DT logger — Digital Entity. On the real robot there is no separate
        # sim odom, so both odom params point at /odom (sync_error_m ~ 0).
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
