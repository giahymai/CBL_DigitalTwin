import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Run SLAM with Cartographer. Real robot bringup must be running first."""
    cart_dir = get_package_share_directory('turtlebot3_cartographer')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cart_dir, 'launch', 'cartographer.launch.py')
            ),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
    ])
