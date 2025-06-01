import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time
import numpy as np

class CarDetectorNode(Node):
    def __init__(self):
        super().__init__('car_detector_node')
        self.bridge = CvBridge()
        self.model = YOLO("yolov5s.pt")  
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.prev_positions = []
        self.prev_time = None
        self.pixel_to_meter = 0.05  
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]
        current_time = time.time()

        car_centers = []
        for box in results.boxes:
            cls_id = int(box.cls)
            label = self.model.names[cls_id]
            if label in ['car', 'truck', 'bus']:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                car_centers.append((cx, cy))
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)

        if self.prev_time and self.prev_positions:
            dt = current_time - self.prev_time
            for (cx, cy), (pcx, pcy) in zip(car_centers, self.prev_positions):
                dx = cx - pcx
                dy = cy - pcy
                distance_px = np.sqrt(dx**2 + dy**2)
                speed_px_per_sec = distance_px / dt
                speed_m_per_sec = speed_px_per_sec * self.pixel_to_meter
                speed_kmh = speed_m_per_sec * 3.6

                if speed_kmh > 50:  
                    self.get_logger().info(f"🚗 High-speed car detected: {speed_kmh:.2f} km/h")

                cv2.putText(frame, f"{speed_kmh:.1f} km/h", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self.prev_positions = car_centers
        self.prev_time = current_time

        cv2.imshow("Car Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CarDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
