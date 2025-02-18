import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64
from flask import Flask, jsonify, request
from threading import Thread
from flask_cors import CORS
from std_srvs.srv import Trigger

app = Flask(__name__)
CORS(app)

# Variables globales
temperature_value = 0.0
led_state = False
bp1_state = False
servo_angle = 0

class SensorNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Abonnement au topic 'servo_angle' pour récupérer l'angle du servo
        self.create_subscription(Int32, 'servo_angle', self.servo_callback, 10)
        
        # Abonnement au topic pour la température
        self.create_subscription(Float64, 'capt_temp', self.temp_callback, 10)
        
        # Client pour contrôler le servo
        self.cli = self.create_client(Trigger, 'set_servo_state')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service Servo non disponible, attente...')

    def servo_callback(self, msg):
        global servo_angle
        servo_angle = msg.data  # Mise à jour de l'angle du servo
        self.get_logger().info(f"Nouvel angle du servo : {servo_angle}°")

    def temp_callback(self, msg):
        global temperature_value
        temperature_value = msg.data

    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        global servo_angle
        try:
            response = future.result()
            self.get_logger().info(f"Réponse du servo : {response.message}")
            servo_angle = 90 if servo_angle == 0 else 0
        except Exception as e:
            self.get_logger().error(f"Échec de la requête : {str(e)}")

@app.route('/temp')
def get_data():
    return jsonify({'temperature': temperature_value})

@app.route('/servo')
def get_servo():
    return jsonify({'servo_angle': servo_angle})

@app.route('/control_button', methods=['POST'])
def virtual_button():
    global bp1_state
    bp1_state = not bp1_state  # Inversion de l'état du bouton
    node.send_request()  # Appel du service pour modifier l'angle du servo
    return jsonify({'bp1_state': bp1_state})

def ros_thread():
    global node
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    ros_thread = Thread(target=ros_thread, daemon=True)
    ros_thread.start()
    app.run(host='0.0.0.0', port=5000)
