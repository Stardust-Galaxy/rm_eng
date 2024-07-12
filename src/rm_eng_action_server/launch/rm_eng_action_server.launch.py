import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
  ld = LaunchDescription()
  
  rm_eng_action_server_node = Node(
    name='rm_eng_action_server_node',
    package='rm_eng_action_server',
    executable='rm_eng_action_server_node',
    output='screen',
  )
  
  ld.add_action(rm_eng_action_server_node)
  return ld