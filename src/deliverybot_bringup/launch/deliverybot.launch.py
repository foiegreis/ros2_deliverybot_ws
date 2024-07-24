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

    description_dir = get_package_share_directory("deliverybot_description")
    controllers_dir = get_package_share_directory("deliverybot_controllers")
    teleop_dir = get_package_share_directory("deliverybot_teleop")
    localization_dir = get_package_share_directory("deliverybot_localization")
    sensors_dir = get_package_share_directory("deliverybot_sensors")
    mapping_dir = get_package_share_directory("deliverybot_mapping")
    bringup_dir = get_package_share_directory("deliverybot_bringup")

    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    gazebo_config = os.path.join(bringup_dir, 'config', 'gazebo_config.yaml')


    urdf_path_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(description_dir, "urdf", "deliverybot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    
    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty",
        description="World file name"
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(
            bringup_dir,
            "rviz",
            "display_bringup.rviz"
        ),
        description="Absolute path to rviz config"
    )

    world_path = PathJoinSubstitution([
        bringup_dir,
        "worlds",
        PythonExpression(["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    urdf_path = os.path.join(description_dir, "models")
    urdf_path += os.pathsep + os.path.join(get_package_prefix("deliverybot_description"), "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", urdf_path)

    rviz_config = LaunchConfiguration("rviz_config")
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True}
        ]   
    )

    # Gazebo
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_config
        }.items(),
    )
    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "my_robot",
            "-topic", "robot_description"
        ],
        output="screen"
    )

    # Rviz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': True}],
        arguments=[
            "-d", rviz_config
        ]
    )

    # Lidar fusion 
    lidar_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_dir, "launch", "sensors.launch.py")
        )
    )


    # Teleop and twistmux
    teleoperation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_dir, "launch", "teleop.launch.py")
        ),
        launch_arguments={
            "use_sim_time": 'true'
        }.items()
    )
    
    # Twistmux speed monitoring
    twistmux_speed_monitoring = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_dir, "launch", "twistmux_speed_monitoring.launch.py")
            )
        )

    # Convert twistmux output to TwistStamped
    twist_to_twist_stamped_and_safety_node = Node(
            package='deliverybot_teleop',
            executable='twist_to_twist_stamped_and_safety.py',
            name='twist_controller_stamped',
            parameters=[
                {'input_unstamped_topic' : 'cmd_vel_ack'},
                {'output_stamped_topic': 'cmd_vel_ack_stamped'},
                {'frame_id':'base_link'},
                {'safety_stop_topic': 'safety_stop'},
                {'safety_warning_topic': 'safety_warning'},
            ]
        )

    # Controllers
    ackermann_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controllers_dir, "launch", "controllers.launch.py")
        )
    )

    # EKF Local Localization - Odom from Ackermann + sensor fusion IMU
    local_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_dir, "launch", "local_localization.launch.py")
        )
    )

    # Graph SLAM 
    graph_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_dir, "launch", "graph_slam.launch.py")
        )
    )
   
    
    return LaunchDescription([
    
        env_var,
        urdf_path_arg,
        world_name_arg,
        rviz_config_arg,

        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,

        TimerAction(
            actions=[
                rviz2,
                lidar_fusion,
                teleoperation,
                twistmux_speed_monitoring,
                twist_to_twist_stamped_and_safety_node,
                ackermann_control,
                #local_localization
                graph_slam
                
                
            ],
            period=3.0  
        )
    ])


