import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class LED(Node):

    def __init__(self):
        super().__init__('led')
        self.subscription = self.create_subscription(
            Bool,
            'bp1',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ledState = 0

    def listener_callback(self, msg):
    	self.ledState = msg.data
    	self.get_logger().info('Etat LED : "%s"' % self.ledState)


def main(args=None):
    rclpy.init(args=args)

    led = LED()

    rclpy.spin(led)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    led.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
