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

    #Ackermann Controllers ++++++++++++++++++++++++++++++++++++++++++
    joint_state_broadcaster =Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']

    )
    ackermann_steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller']
    )

    return LaunchDescription([
  
        ackermann_steering_controller,
        joint_state_broadcaster

    ])


