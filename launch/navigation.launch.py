import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch Nav2 + autonomous Farm Twin navigation on the REAL robot (lab),
    with the Gazebo Digital Twin shown alongside RViz2.

    This is the commit-1758379 Nav2 stack (which reliably shows the map in
    RViz2, because it lets turtlebot3_navigation2 use its OWN default Nav2
    params — no custom params_file is forced in), plus the Gazebo twin window.

    This brings up, in ONE command:
      - Gazebo Digital Twin  (visual 3D mirror, opens next to RViz2)
      - Nav2 (turtlebot3_navigation2) with your lab map + AMCL + RViz2
      - nav2_navigator    (drives zones in sequence, auto return-home on low battery)
      - zone_monitor_node (fires /farm_action at each zone)
      - dt_logger_node    (Digital Entity log + /dt/status)

    No twin_safety_node here (its reflex made the real robot lurch near walls/
    zones — it lives in the TELEOP stack farm_twin.launch.py instead). Zone
    layout comes from the current navigator_node / zone_monitor_node source.

    Robot bringup (ros2 launch turtlebot3_bringup robot.launch.py) must already
    be running over SSH, and ROS_DOMAIN_ID must match the robot.

    Pass the map path on the command line. The repo ships the real SLAM scan at
    maps/lab_map.yaml — point map:= at it (source tree or install/share).

    USAGE (real robot, lab):
      ros2 launch farm_twin_poc navigation.launch.py \
          map:=$HOME/turtlebot3_ws/src/farm_twin_poc/maps/lab_map.yaml

    Wait for "Nav2 is active", then:
      ros2 service call /start_navigation std_srvs/srv/Trigger

    Override start/home position if robot is not at (3, 3):
      ros2 launch farm_twin_poc navigation.launch.py home_x:=X home_y:=Y \
          map:=<path>

    ARGS:
      map                     path to map yaml  (pass map:=<path>; default ~/map.yaml)
      use_sim_time            false on real robot (default: false)
      home_x, home_y, home_yaw  robot start pose + Gazebo spawn (default: 3, 3, 0)
      return_battery_percent  low-battery return threshold % (default: 20)
      set_initial_pose        seed AMCL automatically (default: true)
    """
    pkg_nav2  = get_package_share_directory('turtlebot3_navigation2')
    pkg_share = get_package_share_directory('farm_twin_poc')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    home_x       = LaunchConfiguration('home_x')
    home_y       = LaunchConfiguration('home_y')
    home_yaw     = LaunchConfiguration('home_yaw')
    ret_batt     = LaunchConfiguration('return_battery_percent')
    set_pose     = LaunchConfiguration('set_initial_pose')

    return LaunchDescription([
        # Pass the map path yourself on the command line, e.g.
        #   map:=$HOME/turtlebot3_ws/src/farm_twin_poc/maps/lab_map.yaml
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/map.yaml'),
            description='Path to map yaml — pass map:=/abs/path/lab_map.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Default start position matches the fixed robot placement, the Gazebo
        # spawn, and the current zone layout (robot starts top-left near zone C).
        DeclareLaunchArgument('home_x', default_value='3.0'),
        DeclareLaunchArgument('home_y', default_value='3.0'),
        DeclareLaunchArgument('home_yaw', default_value='0.0'),
        DeclareLaunchArgument('return_battery_percent', default_value='20.0'),
        DeclareLaunchArgument('set_initial_pose', default_value='true'),

        # 1) Gazebo Digital Twin — opens alongside RViz2. Spawns at home_x/home_y
        # so the twin starts at the same map position as the physical robot.
        # gazebo_twin namespaces its topics under /sim (so /sim/scan is kept
        # separate from the real robot's /scan that Nav2 uses).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'gazebo_twin.launch.py')
            ),
            launch_arguments={
                'x_pose': home_x,
                'y_pose': home_y,
            }.items(),
        ),

        # 2) Nav2 + AMCL with the lab map. NOTE: no params_file is passed, so
        # turtlebot3_navigation2 uses its own default Nav2 params + RViz config —
        # this is the configuration under which the map reliably renders in RViz2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'map':          map_yaml,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # 3) Nav2 navigator (zones in sequence + return-home). Zone coordinates
        # come from navigator_node.py (current layout). At each zone the robot
        # just STOPS and dwells — pause keeps localization steady.
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
                'zone_signal':            'pause',
                'spin_cmd_topic':         '/cmd_vel',
                'use_sim_time':           use_sim_time,
            }],
        ),

        # 4) Zone monitor — fires /farm_action at each zone. Detect in the MAP
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

        # 5) DT logger — Digital Entity. On the real robot there is no separate
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
