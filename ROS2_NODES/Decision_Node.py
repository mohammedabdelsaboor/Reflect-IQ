
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.car_danger = False
        self.ultrasonic_dist = 1.0
        self.publisher_ = self.create_publisher(String, '/car_command', 10)

        self.create_subscription(Bool, '/car_speed_alert', self.car_alert_cb, 10)
        self.create_subscription(Float32, '/ultrasonic_distance', self.ultrasonic_cb, 10)
        self.timer = self.create_timer(0.1, self.decide_action)

    def car_alert_cb(self, msg):
        self.car_danger = msg.data

    def ultrasonic_cb(self, msg):
        self.ultrasonic_dist = msg.data

    def decide_action(self):
        cmd = String()
        if self.car_danger or self.ultrasonic_dist < 0.5:
            cmd.data = "STOP"
        else:
            cmd.data = "FORWARD"
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
