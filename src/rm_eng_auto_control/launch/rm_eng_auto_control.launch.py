import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
  ld = LaunchDescription()
  
  rm_eng_auto_control_node = Node(
    name='rm_eng_auto_control_node',
    package='rm_eng_auto_control',
    executable='rm_eng_auto_control_node',
    output='screen',
  )
  
  ld.add_action(rm_eng_auto_control_node)
  return ld