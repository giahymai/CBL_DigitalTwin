import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch Nav2 for autonomous navigation.
    Requires map.yaml from SLAM.

    Usage (real robot):
      ros2 launch farm_twin_poc navigation.launch.py map:=~/map.yaml

    After launching: set "2D Pose Estimate" in RViz, then:
      ros2 service call /start_navigation std_srvs/srv/Trigger
    """
    pkg_nav2 = get_package_share_directory('turtlebot3_navigation2')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/map.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'map':          LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
