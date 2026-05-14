from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    farm_twin.launch.py — Full Digital Twin system (Nodes 2+3+4+5).

    Run AFTER:
      T1: robot bringup (SSH)
      T2: ros2 launch farm_twin_poc gazebo_twin.launch.py
      T3: ros2 launch farm_twin_poc navigation.launch.py map:=~/map.yaml  (optional)
      T4: THIS FILE
      T5: teleop
    """
    return LaunchDescription([
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
        Node(
            package='farm_twin_poc',
            executable='zone_monitor_node',
            name='zone_monitor_node',
            output='screen',
        ),
        Node(
            package='farm_twin_poc',
            executable='dt_logger_node',
            name='dt_logger_node',
            output='screen',
        ),
        Node(
            package='farm_twin_poc',
            executable='navigator_node',
            name='navigator_node',
            output='screen',
        ),
    ])
