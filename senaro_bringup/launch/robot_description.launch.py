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

    start_insta360_cmd = ExecuteProcess(
        cmd=["ros2", "run", "insta360_node", "insta360_node"], output="screen"
    )

    start_um7_cmd = ExecuteProcess(
        cmd=["ros2", "run", "um7", "um7_node"], output="screen"
    )
    base_to_laser_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0.1', '-3.141592', '0', '0', "base_link", "laser"], output="screen"
    )
    odom_to_base_publisher = ExecuteProcess(
        cmd=["ros2", "run", "tf2_ros", "static_transform_publisher", '0', '0', '0', '0', '0', '0', "base_link", "imu_link"], output="screen"
    )

    start_web_bridge_cmd = ExecuteProcess(
        cmd=["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"], output="screen"
    )

    return LaunchDescription([
        base_to_laser_publisher,
        odom_to_base_publisher,
        start_s2lidar_cmd,
        start_tracer_mini_cmd,
        start_insta360_cmd,
        # start_um7_cmd,
        start_web_bridge_cmd
    ])