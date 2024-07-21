from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    ekf_config = os.path.join(get_package_share_directory('deliverybot_localization'), 'config', 'ekf.yaml')
    
    ekf_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )
    return LaunchDescription([
        ekf_localization
    ])