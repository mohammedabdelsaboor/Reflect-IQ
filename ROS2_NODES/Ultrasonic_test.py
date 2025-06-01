import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.publisher_ = self.create_publisher(Float32, '/ultrasonic_distance', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
        self.timer = self.create_timer(0.1, self.read_distance)

    def read_distance(self):
        if self.serial_port.in_waiting:
            line = self.serial_port.readline().decode().strip()
            try:
                distance = float(line)
                msg = Float32()
                msg.data = distance
                self.publisher_.publish(msg)
            except ValueError:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
