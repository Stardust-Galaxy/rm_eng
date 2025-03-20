import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_to_reference_pitch_arg = DeclareLaunchArgument(
        'camera_to_reference_pitch',
        default_value='0.0',
        description='Camera to reference pitch'
    )
    camera_to_reference_yaw_arg = DeclareLaunchArgument(
        'camera_to_reference_yaw',
        default_value='0.0',
        description='Camera to reference yaw'
    )
    camera_to_reference_roll_arg = DeclareLaunchArgument(
        'camera_to_reference_roll',
        default_value='0.0',
        description='Camera to reference roll'
    )
    red_threshold_arg = DeclareLaunchArgument(
        'red_threshold',
        default_value='100',
        description='Threshold value for red color detection'
    )

    blue_threshold_arg = DeclareLaunchArgument(
        'blue_threshold',
        default_value='80',
        description='Threshold value for blue color detection'
    )

    min_area_arg = DeclareLaunchArgument(
        'min_area',
        default_value='1000.0',
        description='Minimum area for contour detection'
    )

    max_area_arg = DeclareLaunchArgument(
        'max_area',
        default_value='10000.0',
        description='Maximum area for contour detection'
    )

    min_small_square_area_arg = DeclareLaunchArgument(
        'min_small_square_area',
        default_value='300.0',
        description='Minimum area for small square detection'
    )

    max_small_square_area_arg = DeclareLaunchArgument(
        'max_small_square_area',
        default_value='1000.0',
        description='Maximum area for small square detection'
    )

    detect_blue_arg = DeclareLaunchArgument(
        'detect_blue_color',
        default_value='false',
        description='Detect blue color'
    )

    front_sign_detector_node = Node(
        name='front_sign_detector_node',
        package='front_sign_detector',
        executable='front_sign_detector_node',
        output='screen',
        parameters=[{
            'red_threshold': LaunchConfiguration('red_threshold'),
            'blue_threshold': LaunchConfiguration('blue_threshold'),
            'min_area': LaunchConfiguration('min_area'),
            'max_area': LaunchConfiguration('max_area'),
            'min_small_square_area': LaunchConfiguration('min_small_square_area'),
            'max_small_square_area': LaunchConfiguration('max_small_square_area'),
            'detect_blue_color': LaunchConfiguration('detect_blue_color'),
            'camera_to_reference_pitch': LaunchConfiguration('camera_to_reference_pitch'),
            'camera_to_reference_yaw': LaunchConfiguration('camera_to_reference_yaw'),
            'camera_to_reference_roll': LaunchConfiguration('camera_to_reference_roll')
        }]
    )

    return LaunchDescription([
        red_threshold_arg,
        blue_threshold_arg,
        min_area_arg,
        max_area_arg,
        min_small_square_area_arg,
        max_small_square_area_arg,
        detect_blue_arg,
        camera_to_reference_pitch_arg,
        camera_to_reference_yaw_arg,
        camera_to_reference_roll_arg,
        front_sign_detector_node
    ])