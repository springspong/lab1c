import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from custom_msgs.msg import OpenSpace


class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.scan_callback,
            10)
        self.subscription  
        self.publisher_open_space = self.create_publisher(
            OpenSpace, 
            'open_space', 
            10)  # Create publisher for OpenSpace message
        self.get_logger().info('OpenSpacePublisher node has been started.')

    def scan_callback(self, msg):
        max_range = max(msg.ranges)
        max_index = msg.ranges.index(max_range)
        max_angle = msg.angle_min + max_index * msg.angle_increment

        open_space_msg = OpenSpace()  # Create an instance of OpenSpace message
        open_space_msg.angle = max_angle
        open_space_msg.distance = max_range

        self.publisher_open_space.publish(open_space_msg) 


def main(args=None):
    rclpy.init(args=args)

    open_space_publisher = OpenSpacePublisher()

    rclpy.spin(open_space_publisher)

    open_space_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

