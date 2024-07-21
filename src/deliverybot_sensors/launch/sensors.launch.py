
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    lidar_fusion_node = Node(
        package="deliverybot_sensors",
        executable="lidar_fusion_node.py",
        name="lidar_fusion_node"
    ) 

    return LaunchDescription([
        lidar_fusion_node
    ])
