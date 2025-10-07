#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')

        # Subscribe to the compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.listener_callback,
            10
        )

        self.get_logger().info('Subscribed to /camera/image/compressed')

    def listener_callback(self, msg):
        # Convert the compressed image data to a NumPy array
        np_arr = np.frombuffer(msg.data, np.uint8)

        # Decode the image
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            # Display the image
            cv2.imshow("Received Image", frame)
            cv2.waitKey(1)  # Needed to render image
        else:
            self.get_logger().warn("Failed to decode image")

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

