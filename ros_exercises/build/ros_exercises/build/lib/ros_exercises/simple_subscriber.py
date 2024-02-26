#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'my_random_float',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Float32, 'random_float_log', 10)

    def listener_callback(self, msg):
        # Calculate natural log of the received message
        log_msg = Float32()
        log_msg.data = math.log(msg.data)
        self.publisher.publish(log_msg)
        self.get_logger().info('Received and published log of random float: %f' % log_msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

