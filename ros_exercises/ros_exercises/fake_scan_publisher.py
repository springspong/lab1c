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
        
        # Retrieve parameters
        self.declare_parameter('angle_min', -2/3 * math.pi)
        self.declare_parameter('angle_max', 2/3 * math.pi)
        self.declare_parameter('angle_increment', 1/300 * math.pi)
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('publish_rate', 20)  # Default publish rate of 20 Hz
        
        # Get parameter values
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        scan = LaserScan()
        scan.header = header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

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
