import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import math

class PointCloudSubscriber(Node):
    def __init__(self):
        # Initialize the node with the name 'point_cloud_subscriber'
        super().__init__('point_cloud_subscriber')

        # Create a subscription to the point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,                    # Message type
            '/camera/camera/depth/color/points',   # Topic name
            self.listener_callback,         # Callback function
            10                              # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        """
        Callback function executed whenever a new PointCloud2 message is received.
        Extracts the 3D coordinates of the center pixel from the point cloud.
        """
        # Get point step (bytes per point) and dimensions of the point cloud
        point_step = msg.point_step
        width = msg.width
        height = msg.height

        # Calculate the index for the center pixel
        x_pixel = width // 2  # Integer division for x-coordinate
        y_pixel = height // 2  # Integer division for y-coordinate
        index = y_pixel * width + x_pixel  # Row-major index in the 1D data array

        # Calculate the byte offset for the desired point
        offset = index * point_step

        # Ensure the offset is within the data buffer
        if offset + point_step <= len(msg.data):
            # Dynamically find the offsets for x, y, z fields in the point cloud
            x_offset = next(field.offset for field in msg.fields if field.name == 'x')
            y_offset = next(field.offset for field in msg.fields if field.name == 'y')
            z_offset = next(field.offset for field in msg.fields if field.name == 'z')

            # Extract the byte sequences for x, y, z (each is a 4-byte float)
            x_bytes = bytes(msg.data[offset + x_offset:offset + x_offset + 4])
            y_bytes = bytes(msg.data[offset + y_offset:offset + y_offset + 4])
            z_bytes = bytes(msg.data[offset + z_offset:offset + z_offset + 4])

            # Unpack the bytes into float values (little-endian format)
            x = struct.unpack('<f', x_bytes)[0]
            y = struct.unpack('<f', y_bytes)[0]
            z = struct.unpack('<f', z_bytes)[0]

            # Check if the coordinates are valid (not NaN)
            if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                self.get_logger().info(f"3D coordinates of center pixel: ({x:.3f}, {y:.3f}, {z:.3f}) meters")
            else:
                self.get_logger().warn("Center pixel has invalid coordinates (NaN)")
        else:
            self.get_logger().error("Point index out of bounds")

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create an instance of the node
    node = PointCloudSubscriber()

    # Keep the node running and processing callbacks
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()