import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Float32, 'my_random_float', 10)
        self.timer_ = self.create_timer(1.0/20.0, self.publish_random_number)

    def publish_random_number(self):
        msg = Float32()
        msg.data = random.uniform(0, 10)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

