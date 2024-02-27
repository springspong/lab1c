import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Float32
import random
import math


class FakeScanPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'fake_scan', 10)
        self.range_test_ = self.create_publisher(Float32, 'range_test', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        scan = LaserScan()
        scan.header = header
        scan.angle_min = -2/3 * math.pi
        scan.angle_max = 2/3 * math.pi
        scan.angle_increment = 1/300 * math.pi
        scan.range_min = 1.0
        scan.range_max = 10.0

        # Generate random ranges
        num_readings = int(abs((scan.angle_max - scan.angle_min)) / scan.angle_increment + 1)
        scan.ranges = [random.uniform(scan.range_min, scan.range_max) for _ in range(num_readings)]

        self.publisher_.publish(scan)
        range_length = Float32()
        range_length.data = float(len(scan.ranges))
        self.range_test_.publish(range_length)

def main(args=None):
    rclpy.init(args=args)

    fake_scan_publisher = FakeScanPublisher()

    rclpy.spin(fake_scan_publisher)

    fake_scan_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

