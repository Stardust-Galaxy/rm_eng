import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    side_sign_detector_node = Node(
        name='side_sign_detector_node',
        package='side_sign_detector',
        executable='side_sign_detector_node',
        output='screen',
    )

    ld.add_action(side_sign_detector_node)
    return ld