import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.serial = serial.Serial('/dev/ttyUSB1', 9600)
        self.subscription = self.create_subscription(String, '/car_command', self.cmd_cb, 10)

    def cmd_cb(self, msg):
        self.serial.write((msg.data + '\n').encode())


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
