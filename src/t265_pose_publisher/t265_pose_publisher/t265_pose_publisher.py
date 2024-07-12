import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import pyrealsense2 as rs

class T265PosePublisher(Node):
    def __init__(self):
        super().__init__('t265_pose_publisher')

        # RealSense pipeline configuration
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.pose)
        self.pipeline.start(config)

        # ROS2 publisher
        self.pose_publisher = self.create_publisher(PoseStamped, 'T265_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)

    def publish_pose(self):
        frames = self.pipeline.wait_for_frames()
        pose_frame = frames.get_pose_frame()
        if pose_frame:
            pose_data = pose_frame.get_pose_data()
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = pose_data.translation.x
            pose_msg.pose.position.y = pose_data.translation.y
            pose_msg.pose.position.z = pose_data.translation.z
            pose_msg.pose.orientation.x = pose_data.rotation.x
            pose_msg.pose.orientation.y = pose_data.rotation.y
            pose_msg.pose.orientation.z = pose_data.rotation.z
            pose_msg.pose.orientation.w = pose_data.rotation.w
            self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = T265PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()