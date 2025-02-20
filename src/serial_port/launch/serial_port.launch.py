import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
  ld = LaunchDescription()
  
  serial_port_node = Node(
    name='serial_port_node',
    package='serial_port',
    executable='serial_port_node',
    respawn=True,
    output='screen',
  )
  
  ld.add_action(serial_port_node)
  return ld