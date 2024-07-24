import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    #Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(get_package_share_directory("deliverybot_mapping"), "config", "slam_toolbox_config.yaml")
    )

    #Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")
    
    #Lifecycle nodes
    lifecycle_nodes = ['map_saver_server']

    #Slam toolbox node
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    #Store map node
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {'save_map_timeout': 5.0},
            {'use_sim_time': use_sim_time},
            {'free_thresh_default': 0.196},
            {'occupied_thresh_default': 0.65}
        ]
    )

    #Nav2 Lifecycle manager
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {'node_names': lifecycle_nodes},
            {'use_sim_time': use_sim_time},
            {'autostart': True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        slam_toolbox,
        nav2_map_saver,
        nav2_lifecycle_manager
    ])