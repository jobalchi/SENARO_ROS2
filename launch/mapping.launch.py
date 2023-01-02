# cartographer.launch.py
 
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
 
 
def generate_launch_description():
    package_name = "senaro"

    pkg_path = os.path.join(get_package_share_directory(package_name))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # cartographer package
    cartographer_prefix = get_package_share_directory('cartographer_ros')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  pkg_path, 'configuration'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_2d.lua')
 
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # s2lidar package
    s2lidar_prefix = get_package_share_directory('sllidar_ros2')
    start_s2lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(s2lidar_prefix, 'launch', 'sllidar_s2_launch.py'))
    )

    # tracer mini package
    tracer_prefix = get_package_share_directory('tracer_base')
    start_tracer_mini_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tracer_prefix, 'launch', 'tracer_mini_base.launch.py'))
    )

    base_to_laser_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0.1', '0', '0', '0', "base_link", "laser"], output="screen"
    )
    odom_to_base_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0', '0', '0', '0', "base_link", "imu_link"], output="screen"
    )

    rviz_config = os.path.join(pkg_path, "configuration", "mapping_rviz.rviz")
 
    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            node_name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),
 
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
 
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),
        base_to_laser_publisher,
        odom_to_base_publisher,
        start_s2lidar_cmd,
        start_tracer_mini_cmd, 
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            parameters=[{'use_sim_time': use_sim_time, 'd': rviz_config}],
            output='screen'),
    ])