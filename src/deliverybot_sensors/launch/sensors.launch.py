
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pointcloud_fusion_node = Node(
        package="deliverybot_sensors",
        executable="pointcloud_fusion_node.py",
        name="pointcloud_fusion"
    ) 

    pointcloud_to_laserscan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/fused_pointcloud'),
                ('scan', '/fused_scan'),
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.2,
                'max_height': 0.5,
                'angle_min': -3.14,  # -M_PI/2
                'angle_max': 3.14,   # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'qos_overrides./fused_scan.reliability': 'best_effort',
                'qos_overrides./fused_scan.durability': 'volatile',
            }]
        )
    
    #NB: the qos remapping doesn't work. In rviz, set reliability=best_effort, durability=volatile

    return LaunchDescription([
        pointcloud_fusion_node,
        pointcloud_to_laserscan,

    ])

