import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
  ld = LaunchDescription()
  
  goal_joint_state_publisher_node = Node(
    name='goal_joint_state_publisher_node',
    package='goal_joint_state_publisher',
    executable='goal_joint_state_publisher_node',
    output='screen',
  )
  
  ld.add_action(goal_joint_state_publisher_node)
  return ld