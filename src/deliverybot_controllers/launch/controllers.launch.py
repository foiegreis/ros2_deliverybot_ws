from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python import get_package_share_directory

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")


    #Ackermann Controllers ++++++++++++++++++++++++++++++++++++++++++
    joint_state_broadcaster =Node(
        package='controller_manager',
        executable='spawner',
        output = 'screen',
        arguments=['joint_state_broadcaster',
                  "--controller-manager", 
                   "/controller_manager"],
       

    )
    ackermann_steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        output = 'screen',
        arguments=['ackermann_steering_controller',
                  "--controller-manager", 
                   "/controller_manager" ],
        
    )


    return LaunchDescription([
        use_sim_time_arg,

        joint_state_broadcaster,
        ackermann_steering_controller

        
    ])


