from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """
    farm_twin.launch.py — Full Digital Twin system (Nodes 2+3+4+5)

    AT HOME (Gazebo simulation):
      ros2 launch farm_twin_poc farm_twin.launch.py

    AT LAB (real robot):
      ros2 launch farm_twin_poc farm_twin.launch.py lab:=true

    The 'lab' argument switches all topic names automatically:
      lab:=false  → /sim/scan, /sim/odom  (Gazebo topics)
      lab:=true   → /scan, /odom          (real robot topics)
    """
    lab_arg = DeclareLaunchArgument(
        'lab',
        default_value='false',
        description='false = at home (Gazebo) | true = lab session (real robot)',
    )
    lab = LaunchConfiguration('lab')

    return LaunchDescription([
        lab_arg,

        # Node 2: Twin Safety
        Node(
            package='farm_twin_poc',
            executable='twin_safety_node',
            name='twin_safety_node',
            parameters=[{
                'real_scan_topic': '/scan',
                'sim_scan_topic':  '/sim/scan',
                'input_cmd_topic': '/cmd_vel_raw',
                'real_cmd_topic':  '/cmd_vel',
                'sim_cmd_topic':   '/sim/cmd_vel',
                'stop_distance':   0.25,
                'front_angle_deg': 30.0,
            }],
            output='screen',
        ),

        # Node 3: Zone Monitor — at home uses /sim/odom, at lab uses /odom
        Node(
            package='farm_twin_poc',
            executable='zone_monitor_node',
            name='zone_monitor_node',
            parameters=[{'odom_topic': '/sim/odom'}],
            condition=UnlessCondition(lab),
            output='screen',
        ),
        Node(
            package='farm_twin_poc',
            executable='zone_monitor_node',
            name='zone_monitor_node',
            parameters=[{'odom_topic': '/odom'}],
            condition=IfCondition(lab),
            output='screen',
        ),

        # Node 4: Digital Entity Logger
        # Tracks both real_position and sim_position → sync_error_m shows how well they match
        Node(
            package='farm_twin_poc',
            executable='dt_logger_node',
            name='dt_logger_node',
            parameters=[{
                'real_odom_topic': '/sim/odom',  # at home: Gazebo is the "real"
                'sim_odom_topic':  '/sim/odom',  # same source
            }],
            condition=UnlessCondition(lab),
            output='screen',
        ),
        Node(
            package='farm_twin_poc',
            executable='dt_logger_node',
            name='dt_logger_node',
            parameters=[{
                'real_odom_topic': '/odom',      # at lab: real robot
                'sim_odom_topic':  '/sim/odom',  # Gazebo twin
            }],
            condition=IfCondition(lab),
            output='screen',
        ),

        # Node 5: Navigator — at home uses /sim/*, at lab uses real topics
        Node(
            package='farm_twin_poc',
            executable='navigator_node',
            name='navigator_node',
            parameters=[{
                'scan_topic':        '/sim/scan',
                'odom_topic':        '/sim/odom',
                'battery_topic':     '/battery_state',
                'max_linear':        0.15,
                'max_angular':       0.6,
                'goal_tolerance':    0.25,
                'obstacle_distance': 0.40,
                'front_angle_deg':   45.0,
            }],
            condition=UnlessCondition(lab),
            output='screen',
        ),
        Node(
            package='farm_twin_poc',
            executable='navigator_node',
            name='navigator_node',
            parameters=[{
                'scan_topic':        '/scan',
                'odom_topic':        '/odom',
                'battery_topic':     '/battery_state',
                'max_linear':        0.15,
                'max_angular':       0.6,
                'goal_tolerance':    0.25,
                'obstacle_distance': 0.40,
                'front_angle_deg':   45.0,
            }],
            condition=IfCondition(lab),
            output='screen',
        ),
    ])