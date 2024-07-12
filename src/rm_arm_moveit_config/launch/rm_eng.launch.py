import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
    robot_name_in_model = 'rm_arm_description'
    package_name = 'rm_arm_description'
    urdf_name = "rm_arm_description.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # 因为 urdf文件中有一句 $(find mybot) 需要用xacro进行编译一下才行;否则就是非法文件了
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容
    # 并且会订阅 /joint_states 话题，获取关节的数据，然后发布tf和tf_static话题； 这三个话题在该节点启动时就会出现
    # 这些节点、话题的名称可不可以自定义？
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        respawn=True,
        # parameters=[params, ],
        parameters=[params, {"publish_frequency": 15.0}],
        output='screen'
    )

    # 对于自己写的机械手控制，需要自己实现action、joint_states的发布；同时，也就不需要使用ros2_control了，因为它是用来实现前面提到的那两个的
    # 因此，这边只需要启动一个robot_state_publisher即可

    ld = LaunchDescription()

    ld.add_action(node_robot_state_publisher)

    return ld
