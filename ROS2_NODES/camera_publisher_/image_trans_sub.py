
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import time
from threading import Thread, Lock

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscribed to /camera/image/compressed')

        self.prev_time = time.time()
        self.fps = 0.0

        # FPS box settings
        self.box_position = (10, 10)
        self.box_width = 120
        self.box_height = 30
        self.font_scale = 0.6
        self.font_thickness = 2

        # Threading for smooth display
        self.frame = None
        self.lock = Lock()
        self.running = True
        self.display_thread = Thread(target=self.display_frame)
        self.display_thread.start()

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            current_time = time.time()
            dt = current_time - self.prev_time
            if dt > 0:
                self.fps = 1.0 / dt
            self.prev_time = current_time

            x, y = self.box_position
            cv2.rectangle(frame, (x, y), (x + self.box_width, y + self.box_height),
                          (0, 0, 0), -1)
            cv2.putText(frame, f"FPS: {self.fps:.2f}", (x + 5, y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (0, 255, 0), self.font_thickness)

            with self.lock:
                self.frame = frame
        else:
            self.get_logger().warn("Failed to decode image")

    def display_frame(self):
        while self.running:
            frame_copy = None
            with self.lock:
                if self.frame is not None:
                    frame_copy = self.frame.copy()
            if frame_copy is not None:
                cv2.imshow("Received Image", frame_copy)
                cv2.waitKey(1)

    def __del__(self):
        self.running = False
        self.display_thread.join()
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
