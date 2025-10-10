import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_pub_node')

        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 60)

        if not self.cap.isOpened():
            self.get_logger().error("❌ Camera not opened! Check /dev/video0 or permissions.")
        else:
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f"✅ Camera opened successfully @ {fps:.1f} fps")

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback) 

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Frame not captured.")
            return

        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
