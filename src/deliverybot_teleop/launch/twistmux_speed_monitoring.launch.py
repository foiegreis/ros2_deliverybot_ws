from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, LogInfo
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Declare arguments ++++++++++++++++++++++++++++++++++
    danger_distance_arg = DeclareLaunchArgument(
        "danger_distance",
        default_value= "2.6",
        description="Danger distance for safety stop"
    )   

    warning_distance_arg = DeclareLaunchArgument(
        "warning_distance",
        default_value= "3.8",
        description="Warning distance for safety stop"
    )   

    # Launch arguments ++++++++++++++++++++++++++++++++++
    danger_distance = LaunchConfiguration('danger_distance')
    warning_distance = LaunchConfiguration('warning_distance')

    # TwistMux Speed Monitoring +++++++++++++++++++++++++++++++++++++++++++++

    
    twistmux_safety_stop = Node(
        package="deliverybot_teleop",
        executable="twistmux_speed_monitoring",
        name="twistmux_speed_monitoring",
        parameters=[
            {'danger_distance' : danger_distance},
            {'warning_distance': warning_distance},
            {'pcl_topic' : 'fused_pointcloud'},
            {'safety_stop_topic' : 'safety_stop'},
            {'safety_warning_topic': 'safety_warning'},
            {'markers_reference_link': 'base_link'}
        ]
    )

    return LaunchDescription([

        LogInfo(msg=f"Launching Twistmux"),

        danger_distance_arg,
        warning_distance_arg,
        twistmux_safety_stop

        

    ])
