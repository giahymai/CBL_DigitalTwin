import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch Nav2 + autonomous Farm Twin navigation on the REAL robot (lab),
    with the Gazebo Digital Twin shown alongside RViz2.

    This is the proven commit-1758379 Nav2 stack (turtlebot3_navigation2 with its
    OWN default Nav2 params + RViz config — the setup under which the map reliably
    renders), PLUS the Gazebo twin window.

    CRUCIAL: gazebo_twin.launch.py does PushRosNamespace('sim'). If it is included
    in the same scope as Nav2, that namespace LEAKS onto Nav2 — the whole Nav2
    stack (map_server, amcl, costmaps, rviz) ends up under /sim, listening to
    /sim/scan and /sim/tf, which do NOT carry the real robot's data (the robot
    publishes /scan and /tf at the GLOBAL namespace). The result is a blank map
    and "base_scan ... queue is full" errors. So the Gazebo include is wrapped in
    a SCOPED GroupAction here, which contains the /sim namespace inside Gazebo and
    keeps Nav2 in the global namespace where it can see the real robot.

    This brings up, in ONE command:
      - Gazebo Digital Twin  (visual 3D mirror, opens next to RViz2, topics in /sim)
      - Nav2 (turtlebot3_navigation2) with your lab map + AMCL + RViz2 (GLOBAL ns)
      - nav2_navigator    (drives zones in sequence, auto return-home on low battery)
      - zone_monitor_node (fires /farm_action at each zone)
      - dt_logger_node    (Digital Entity log + /dt/status)

    No twin_safety_node here (its reflex made the real robot lurch — it lives in
    the TELEOP stack farm_twin.launch.py). Zone layout comes from the current
    navigator_node / zone_monitor_node source.

    Robot bringup (ros2 launch turtlebot3_bringup robot.launch.py) must already
    be running over SSH, and ROS_DOMAIN_ID must match the robot.

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
    nav2_params  = LaunchConfiguration('params_file')
    twin_entity  = LaunchConfiguration('twin_entity')

    default_params = os.path.join(pkg_share, 'config', 'nav2_lab.yaml')

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
        # Lab-tuned Nav2 params (transform_tolerance 1.0 for WiFi TF latency,
        # AMCL recovery, collision_monitor timeout) — these prevent the
        # mid-navigation "goal aborted" that happens with the tight default
        # transform tolerances over WiFi.
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 params yaml (default: nav2_lab.yaml)'),
        # gz MODEL name of the Gazebo twin that twin_pose_sync_node teleports to
        # the real robot's pose. Verify with `gz model --list` and override if
        # your spawn names it differently: twin_entity:=burger
        DeclareLaunchArgument('twin_entity', default_value='turtlebot3_burger',
                              description='gz model name of the Gazebo twin to pin'),

        # 1) Gazebo Digital Twin — opens alongside RViz2. WRAPPED in a scoped
        # GroupAction so gazebo_twin's PushRosNamespace('sim') stays INSIDE the
        # group and does NOT leak onto Nav2 below. Gazebo's own topics live under
        # /sim (/sim/scan etc.), kept separate from the real robot's /scan.
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'gazebo_twin.launch.py')
                ),
                launch_arguments={
                    'x_pose': home_x,
                    'y_pose': home_y,
                }.items(),
            ),
        ]),

        # 2) Nav2 + AMCL with the lab map, in the GLOBAL namespace (so it sees the
        # real robot's /scan and /tf). Lab-tuned params_file (nav2_lab.yaml) is
        # passed in to buffer WiFi TF latency and stop mid-navigation aborts.
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

        # 2b) Twin Pose Sync — pin the Gazebo twin to the real robot's localized
        # pose (TF map->base_link) so the Gazebo robot mirrors RViz2 / the real
        # robot. Runs in the GLOBAL namespace so it reads the real robot's TF;
        # it only teleports the gz model, never touches /cmd_vel.
        Node(
            package='farm_twin_poc',
            executable='twin_pose_sync_node',
            name='twin_pose_sync_node',
            output='screen',
            parameters=[{
                'world_name':   'lab_world',
                'entity_name':  twin_entity,
                'global_frame': 'map',
                'robot_frame':  'base_link',
                'rate_hz':      10.0,
                'use_sim_time': use_sim_time,
            }],
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
