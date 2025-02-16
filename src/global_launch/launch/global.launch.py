import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()
    hik_camera_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/home/engineer/code/rm_eng/src/hik_camera/launch/hik_camera.launch.py'
        )
    )
    serial_port_node = Node(
        name='serial_port_node',
        package='serial_port',
        executable='serial_port_node',
        output='screen',
    )
    robotic_arm_moveit_config_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/home/engineer/code/rm_eng/src/robotic_arm_moveit_config/launch/demo.launch.py'
        )
    )
    rm_eng_action_server_node = Node(
        name='rm_eng_action_server_node',
        package='rm_eng_action_server',
        executable='rm_eng_action_server_node',
        output='screen',
    )
    rm_eng_auto_control_node = Node (
        name='rm_eng_auto_control_node',
        package='rm_eng_auto_control',
        executable='rm_eng_auto_control_node',
        output='screen',
    )
    front_sign_detection_node = Node(
        name='front_sign_detector_node',
        package='front_sign_detector',
        executable='front_sign_detector_node',
        output='screen',
    )
    goal_state_publisher_node = Node(
        name='goal_state_publisher_node',
        package='goal_state_publisher',
        executable='goal_state_publisher_node',
        output='screen',
    )
    ld.add_action(hik_camera_launch)
    ld.add_action(serial_port_node)
    ld.add_action(robotic_arm_moveit_config_node)
    ld.add_action(rm_eng_action_server_node)
    ld.add_action(rm_eng_auto_control_node)
    ld.add_action(front_sign_detection_node)
    ld.add_action(goal_state_publisher_node)
    return ld