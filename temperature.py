import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TempPublisher(Node):

    def __init__(self):
        super().__init__('capt_temp')
        self.publisher_ = self.create_publisher(Float64, 'capt_temp', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.val = 25.5  # Température initiale

    def timer_callback(self):
        msg = Float64()
        msg.data = self.val
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Température: {msg.data}°C')

        # Simulation d'un changement de température
        self.val = 17.5 if self.val >= 30 else self.val + 2.0

def main(args=None):
    rclpy.init(args=args)
    capt_temp = TempPublisher()
    rclpy.spin(capt_temp)
    capt_temp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

