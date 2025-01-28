import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class SERVO(Node):

    def __init__(self):
        super().__init__('servoMoteur')
        self.subscription = self.create_subscription(
            Float64,
            'capt_temp',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.AngleServoDeg = 0

    def listener_callback(self, msg):
    	valTemp = msg.data
    	self.AngleServoDeg = (90*valTemp)/30
    		
    	self.get_logger().info('AngleServo : "%s"' % self.AngleServoDeg)


def main(args=None):
    rclpy.init(args=args)

    servoMot = SERVO()

    rclpy.spin(servoMot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servoMot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
