import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    mine_detector_node = Node(
        name='mine_detector_node',
        package='mine_detector',
        executable='mine_detector_node',
        output='screen',
    )

    ld.add_action(mine_detector_node)
    return ld