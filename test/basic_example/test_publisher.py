# A ROS2 node that publishes a monotonicaly increasing number

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MonotonicallyIncreasingPublisher(Node):
    def __init__(self, freq):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32, '/testing_topic', 10)
        timer_period = 1.0 / freq
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.i += 1

if __name__ == '__main__':
    rclpy.init()
    node = MonotonicallyIncreasingPublisher(freq=10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()