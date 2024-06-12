import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    start_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot'), 'launch', 'gazebo.launch.py'))
    )

    rviz_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot'), 'launch', 'usr_moveit.launch.py'))
    )

    ros_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_serial'), 'launch', 'robot_serial.launch.py'))
    )

    return LaunchDescription([
        start_gazebo_launch,
        rviz_moveit_launch,
        ros_serial_launch
    ])