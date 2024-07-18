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

    controllers_dir_name = "deliverybot_controllers"
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    xbox_config_path = os.path.join(get_package_share_directory('teleop_twist_joy'), 'config', 'xbox.config.yaml')
    
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

    #Keyboard teleop +++++++++++++++++++++++++++++++++++++
    key_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --',
        remappings=[("cmd_vel", "cmd_vel")] 
    )

    twist_keyboard_stamped = Node(
        package=controllers_dir_name,
        executable="twist_keyboard_stamped.py",
        name="twist_keyboard_stamped"
    )

    #Joystick teleop +++++++++++++++++++++++++++++++++++++

    # Joystick teleop configuration
    joy_params = {
        'dev': '/dev/input/js1',
        'deadzone': 0.05,
        'autorepeat_rate': 20.0,
    }
    
    #joy_node + teleop_twist_joy
    teleop_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')
            ),
             launch_arguments={
                'joy_config': 'box',
                'joy_dev': joy_params['dev'],
                'config_filepath': xbox_config_path,
                'cmd_vel': 'cmd_vel'  
            }.items()
        ) 
        
        
    twist_joystick_stamped = Node(
            package=controllers_dir_name,
            executable='twist_joystick_stamped.py',
            name='twist_keyboard_stamped'
        )
    
    
    return LaunchDescription([
  
        ackermann_steering_controller,
        joint_state_broadcaster,

        #key_teleop,
        #twist_keyboard_stamped

        LogInfo(msg=f"Using config file: {xbox_config_path}"),
        teleop_launch,
        twist_joystick_stamped

    ])


