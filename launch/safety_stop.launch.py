from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Mode 1: simulation-only safety stop (at-home testing)."""
    return LaunchDescription([
        Node(
            package='farm_twin_poc',
            executable='safety_stop_node',
            name='safety_stop_node',
            parameters=[{
                'scan_topic':       '/scan',
                'input_cmd_topic':  '/cmd_vel_raw',
                'output_cmd_topic': '/cmd_vel',
                'stop_distance':    0.25,
                'front_angle_deg':  30.0,
            }],
            output='screen',
        ),
    ])
