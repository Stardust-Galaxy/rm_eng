from setuptools import find_packages, setup

package_name = 't265_pose_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dji',
    maintainer_email='2409602366@qq.com',
    description='A package to publish T265 pose data to ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            't265_pose_publisher = t265_pose_publisher.t265_pose_publisher:main',
        ],
    },
)
