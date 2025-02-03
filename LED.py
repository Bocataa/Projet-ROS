import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

class LEDSubscriber(Node):

    def __init__(self):
        super().__init__('led_subscriber')
        
        # Subscriber pour recevoir la température
        self.subscription = self.create_subscription(
            Float64,
            'capt_temp',
            self.listener_callback,
            10)
        self.subscription

        # Publisher pour envoyer l'état de la LED
        self.publisher_ = self.create_publisher(Bool, 'led_state', 10)

    def listener_callback(self, msg):
        # Callback déclenché lorsqu'une nouvelle température est reçue
        temp = msg.data
        led_state = temp >= 23  # Allume la LED si température ≥ 28°C

        # Publication de l'état de la LED
        msg_led = Bool()
        msg_led.data = led_state
        self.publisher_.publish(msg_led)
        state_str = "ON" if led_state else "OFF"
        self.get_logger().info(f'Temp: {temp}°C -> LED: {state_str}')

def main(args=None):
    rclpy.init(args=args)
    led_subscriber = LEDSubscriber()
    rclpy.spin(led_subscriber)
    led_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

