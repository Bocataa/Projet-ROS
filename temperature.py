import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float64

class TempPublisher(Node):

    def __init__(self):
        super().__init__('capt_temp')
        self.publisher_ = self.create_publisher(Float64, 'capt_temp', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.val = 25.5

    def timer_callback(self):
        msg = Float64()
        msg.data = self.val
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        if (self.val <= 30):
             self.val += 2.0
        else:
            self.val = 17.5


def main(args=None):
    rclpy.init(args=args)

    capt_temp = TempPublisher()

    rclpy.spin(capt_temp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    capt_temp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
