import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_pub_node')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.rate = self.create_rate(30)  # 30 FPS
        self.run()

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                continue
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            self.publisher_.publish(msg)
            self.get_logger().info('Published compressed image')
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
