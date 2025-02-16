import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    front_sign_detector_node = Node(
        name='front_sign_detector_node',
        package='front_sign_detector',
        executable='front_sign_detector_node',
        output='screen',
    )

    ld.add_action(front_sign_detector_node)
    return ld