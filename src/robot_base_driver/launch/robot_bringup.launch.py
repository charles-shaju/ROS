import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_base_driver')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # Base Driver Node (Encoders & Motors)
        Node(
            package='robot_base_driver',
            executable='base_driver',
            name='base_driver',
            output='screen'
        ),

        # BNO085 IMU Node
        Node(
            package='robot_base_driver',
            executable='bno085_node',
            name='bno085_node',
            output='screen'
        ),

        # EKF Sensor Fusion Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        )
    ])
