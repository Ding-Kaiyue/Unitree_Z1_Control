from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_serial_node = Node (
        package = "robot_serial",
        executable = "talker_node",
    )
    ld.add_action(robot_serial_node)
    return ld