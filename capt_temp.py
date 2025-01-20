import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float64


class TempPublisher(Node):

    def __init__(self):
        super().__init__('capt_temp')
        self.publisher_ = self.create_publisher(Float64, 'temp', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.5

    def timer_callback(self):
        msg = Float64()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 0.5


def main(args=None):
    rclpy.init(args=args)

    temp = TempPublisher()

    rclpy.spin(temp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    temp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
