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

    USAGE (real robot, lab):
      ros2 launch farm_twin_poc navigation.launch.py \
          map:=~/map.yaml home_x:=0.0 home_y:=0.0

    Then in RViz click "2D Pose Estimate" at the robot's REAL location
    (set_initial_pose stays false on the real robot), wait for "Nav2 is active":
      ros2 service call /start_navigation std_srvs/srv/Trigger

    ARGS:
      map                     path to SLAM lab map yaml      (default: ~/map.yaml)
      use_sim_time            false on real robot            (default: false)
      home_x, home_y, home_yaw  robot start pose to return to (default: 0,0,0)
      return_battery_percent  low-battery return threshold % (default: 20)
      set_initial_pose        seed AMCL automatically        (default: false → use RViz)
    """
    pkg_nav2 = get_package_share_directory('turtlebot3_navigation2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    home_x       = LaunchConfiguration('home_x')
    home_y       = LaunchConfiguration('home_y')
    home_yaw     = LaunchConfiguration('home_yaw')
    ret_batt     = LaunchConfiguration('return_battery_percent')
    set_pose     = LaunchConfiguration('set_initial_pose')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/map.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('home_x', default_value='0.0'),
        DeclareLaunchArgument('home_y', default_value='0.0'),
        DeclareLaunchArgument('home_yaw', default_value='0.0'),
        DeclareLaunchArgument('return_battery_percent', default_value='20.0'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),

        # 1) Nav2 + AMCL with the lab map.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'map':          map_yaml,
                'use_sim_time': use_sim_time,
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

        # 3) Zone monitor — fires /farm_action at each zone (real /odom).
        Node(
            package='farm_twin_poc',
            executable='zone_monitor_node',
            name='zone_monitor_node',
            output='screen',
            parameters=[{'odom_topic': '/odom', 'use_sim_time': use_sim_time}],
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
