from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """
    farm_twin.launch.py — Digital Twin core (Nodes 2+3+4)

    This is the Digital-Twin / state-sync stack, driven by TELEOP. It proves
    Bi-directional Communication and State Synchronisation:
      - twin_safety_node : safe /cmd_vel back to the robot AND identical
                           /sim/cmd_vel to the Gazebo twin (state sync) + safety stop
      - zone_monitor_node: fires /farm_action when the robot enters a farm zone
      - dt_logger_node   : Digital Entity log + /dt/status (sync_error_m)

    Autonomous path planning is NOT here — that is Nav2's job
    (gazebo_nav2_demo.launch.py at home, navigation.launch.py at the lab).

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
    ])