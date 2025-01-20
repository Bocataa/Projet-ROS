import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Bool

class Bp1Publisher(Node):

    def __init__(self):
        super().__init__('bp1')
        self.publisher_ = self.create_publisher(Bool, 'bp1', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = True

    def timer_callback(self):
        msg = Bool()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i = not self.i


def main(args=None):
    rclpy.init(args=args)

    bp1 = Bp1Publisher()

    rclpy.spin(bp1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    temp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
