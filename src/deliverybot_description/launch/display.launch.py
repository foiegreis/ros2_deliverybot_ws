import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    description_dir = get_package_share_directory("deliverybot_description")

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="use simulation (Gazebo) clock if true"
    )


    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(
            description_dir,
            "urdf",
            "deliverybot.urdf.xacro"),
            description = "Absolute path to robot urdf file"
        )
    
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(
            description_dir,
            "rviz",
            "display.rviz"
        ),
        description="Absolute path to rviz config"
    )

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    #xacro to urdf
    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)


    #robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description" : robot_description},
            {'use_sim_time': use_sim_time}
        ]
    ) 

    #joint state publisher gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    #rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            "-d", rviz_config
        ]
    )



    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
        
    ])