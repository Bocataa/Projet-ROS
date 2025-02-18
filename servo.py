import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

class ServoServer(Node):

    def __init__(self):
        super().__init__('servo_server')
        
        # Création du service ROS2
        self.srv = self.create_service(Trigger, 'set_servo_state', self.set_servo_callback)
        
        # Publication de l'angle du servo
        self.angle_servo_pub = self.create_publisher(Int32, 'servo_angle', 10)
        
        self.angle_servo = 0  # Angle initial du servo

    def set_servo_callback(self, request, response):
        # Callback du service : modifie l'angle du servo
        self.angle_servo = 90 if self.angle_servo == 0 else 0
        self.get_logger().info(f"Servo ajusté à {self.angle_servo}°")

        # Publication de l'angle du servo sur un topic
        angle_msg = Int32()
        angle_msg.data = self.angle_servo
        self.angle_servo_pub.publish(angle_msg)

        response.success = True
        response.message = f"Servo ajusté à {self.angle_servo}°"
        return response

def main(args=None):
    rclpy.init(args=args)
    servo_server = ServoServer()
    rclpy.spin(servo_server)
    servo_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
