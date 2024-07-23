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

    #xbox_config_path = os.path.join(get_package_share_directory('teleop_twist_joy'), 'config', 'xbox.config.yaml')
    joy_params = os.path.join(get_package_share_directory("deliverybot_teleop"), 'config', 'joystick_config.yaml') #we override xbox.config.yaml
    twistmux_dir = get_package_share_directory('twist_mux')
    teleop_dir = get_package_share_directory('deliverybot_teleop')

    # Declare arguments ++++++++++++++++++++++++++++++++++
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description="Use sim time arg"
    )  

    # Launch arguments ++++++++++++++++++++++++++++++++++
    use_sim_time = LaunchConfiguration('use_sim_time')

    #Keyboard teleop +++++++++++++++++++++++++++++++++++++

    key_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --',
        remappings=[
            ('/cmd_vel', '/key/cmd_vel')
        ] 
    )

    #Joystick teleop +++++++++++++++++++++++++++++++++++++

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            joy_params,
            {'use_sim_time': use_sim_time}]
    )

    joy_teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        remappings=[
            ('/cmd_vel', '/joy/cmd_vel') #cmd vel for joystick
            ],
        parameters=[
            joy_params,
            {'use_sim_time' : use_sim_time}
            ]
    )

    # TwistMux +++++++++++++++++++++++++++++++++++++++++++++
    
    twistmux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            twistmux_dir, 'launch', 'twist_mux_launch.py')
            ),
            launch_arguments={
                'cmd_vel_out': 'cmd_vel_ack',
                'config_topics': os.path.join(teleop_dir, 'config', 'twistmux_topics.yaml'),
                'config_locks': os.path.join(teleop_dir, 'config', 'twistmux_locks.yaml'),
                'config_joy': os.path.join(teleop_dir, 'config', 'twistmux_joy.yaml')
            }.items()
    )
    
    return LaunchDescription([

        LogInfo(msg=f"Launching Twistmux"),

        use_sim_time_arg,

        key_teleop,
        joy_node,
        joy_teleop_node,

        twistmux_launch
        

    ])
