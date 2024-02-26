import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_distance = self.create_publisher(Float32, 'open_space/distance', 10)
        self.publisher_angle = self.create_publisher(Float32, 'open_space/angle', 10)
        self.get_logger().info('OpenSpacePublisher node has been started.')

    def scan_callback(self, msg):
        max_range = max(msg.ranges)
        max_index = msg.ranges.index(max_range)
        max_angle = msg.angle_min + max_index * msg.angle_increment

        distance_msg = Float32()
        distance_msg.data = max_range
        self.publisher_distance.publish(distance_msg)

        angle_msg = Float32()
        angle_msg.data = max_angle
        self.publisher_angle.publish(angle_msg)


def main(args=None):
    rclpy.init(args=args)

    open_space_publisher = OpenSpacePublisher()

    rclpy.spin(open_space_publisher)

    open_space_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

