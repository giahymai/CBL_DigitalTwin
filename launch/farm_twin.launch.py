from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    farm_twin.launch.py — Full Digital Twin system (Nodes 2+3+4+5).

    AT HOME (Gazebo sim):
      T1: ros2 launch farm_twin_poc gazebo_twin.launch.py
      T2: ros2 launch farm_twin_poc farm_twin.launch.py
      T3: (optional teleop) ros2 run turtlebot3_teleop teleop_keyboard
              --ros-args -r /cmd_vel:=/cmd_vel_raw
      OR start autonomous: ros2 service call /start_navigation std_srvs/srv/Trigger

    AT LAB (real robot):
      T1: ssh robot → ros2 launch turtlebot3_bringup robot.launch.py
      T2: ros2 launch farm_twin_poc gazebo_twin.launch.py
      T3: ros2 launch farm_twin_poc farm_twin.launch.py
      T4: teleop OR ros2 service call /start_navigation std_srvs/srv/Trigger

    Monitor:
      ros2 topic echo /farm_action
      ros2 topic echo /dt/status
      ros2 topic echo /sim/cmd_vel
      ros2 topic echo /navigator/status
      ros2 service call /get_dt_log       std_srvs/srv/Trigger
      ros2 service call /get_twin_status  std_srvs/srv/Trigger
    """
    return LaunchDescription([

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

        # Node 3: Zone Monitor
        Node(
            package='farm_twin_poc',
            executable='zone_monitor_node',
            name='zone_monitor_node',
            parameters=[{
                # AT HOME (Gazebo): '/sim/odom'
                # AT LAB (real robot): '/odom'
                'odom_topic': '/sim/odom',
            }],
            output='screen',
        ),

        # Node 4: Digital Entity Logger
        Node(
            package='farm_twin_poc',
            executable='dt_logger_node',
            name='dt_logger_node',
            output='screen',
        ),

        # Node 5: Autonomous Navigator (no Nav2 needed)
        Node(
            package='farm_twin_poc',
            executable='navigator_node',
            name='navigator_node',
            parameters=[{
                # AT HOME (Gazebo): scan_topic='/sim/scan', odom_topic='/sim/odom'
                # AT LAB (real robot): scan_topic='/scan', odom_topic='/odom'
                'scan_topic':        '/sim/scan',
                'odom_topic':        '/sim/odom',
                'max_linear':        0.15,
                'max_angular':       0.5,
                'goal_tolerance':    0.25,
                'obstacle_distance': 0.45,
                'front_angle_deg':   40.0,
            }],
            output='screen',
        ),
    ])