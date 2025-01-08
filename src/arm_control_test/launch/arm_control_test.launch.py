import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    arm_control_test_node = Node(
        name='arm_control_test_node',
        package='arm_control_test',
        executable='arm_control_test_node',
        output='screen',
    )

    ld.add_action(arm_control_test_node)
    return ld