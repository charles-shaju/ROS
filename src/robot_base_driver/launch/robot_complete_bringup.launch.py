import xacro
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    driver_pkg = get_package_share_directory('robot_base_driver')
    desc_pkg = get_package_share_directory('robot_description')
    
    urdf_file = os.path.join(desc_pkg, 'urdf', 'robot.urdf.xacro')
    ekf_config = os.path.join(driver_pkg, 'config', 'ekf.yaml')
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    base_driver_node = Node(
        package='robot_base_driver',
        executable='base_driver',
        parameters=[{'port': '/dev/ttyACM0', 'baudrate': 115200}]
    )

    bno085_node = Node(
        package='robot_base_driver',
        executable='bno085_node'
    )

    # EKF Node for Sensor Fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        base_driver_node,
        bno085_node,
        ekf_node
    ])
