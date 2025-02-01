from flask import Flask, jsonify, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

# Flask app
app = Flask(__name__)

# Initialisation de ROS2
rclpy.init()

# Variables globales pour stocker les données des capteurs
temperature = 25.5
button_state = False
light_level = 200

# Initialisation des publishers pour la LED et le Servo
led_publisher = None
servo_publisher = None

# Définition des Nodes ROS2 pour lire les données des capteurs et envoyer des commandes
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        
        # Abonnement aux topics des capteurs
        self.create_subscription(Float64, 'temp', self.temp_callback, 10)
        self.create_subscription(Bool, 'bp1', self.bp1_callback, 10)
        self.create_subscription(Float64, 'light', self.light_callback, 10)

    def temp_callback(self, msg):
        global temperature
        temperature = msg.data

    def bp1_callback(self, msg):
        global button_state
        button_state = msg.data

    def light_callback(self, msg):
        global light_level
        light_level = msg.data


# Création d'une instance du Node ROS2
sensor_subscriber = SensorSubscriber()

# Création des publishers pour la LED et le Servo
class ActuatorPublisher(Node):
    def __init__(self):
        super().__init__('actuator_publisher')
        global led_publisher, servo_publisher
        led_publisher = self.create_publisher(Bool, 'bp1', 10)  # Publisher pour la LED
        servo_publisher = self.create_publisher(Float64, 'capt_temp', 10)  # Publisher pour le servo

# Création d'une instance pour publier sur les actionneurs
actuator_publisher = ActuatorPublisher()

@app.route('/')
def get_sensor_data():
    # Retourne les données des capteurs sous forme JSON
    return jsonify({
        'temperature': temperature,
        'button_state': button_state,
        'light_level': light_level
    })

@app.route('/control/led', methods=['POST'])
def control_led():
    # Récupérer l'état de la LED (on/off) depuis la requête HTTP
    led_state = request.json.get('state', False)  # False par défaut
    msg = Bool()
    msg.data = led_state
    led_publisher.publish(msg)
    return jsonify({'status': 'LED state changed', 'state': led_state})

@app.route('/control/servo', methods=['POST'])
def control_servo():
    # Récupérer l'angle du servo depuis la requête HTTP
    angle = request.json.get('angle', 0.0)  # Angle par défaut à 0.0
    msg = Float64()
    msg.data = angle
    servo_publisher.publish(msg)
    return jsonify({'status': 'Servo angle changed', 'angle': angle})

if __name__ == '__main__':
    # Lancer Flask dans un thread à part
    from threading import Thread
    def start_flask():
        app.run(host='0.0.0.0', port=3000)

    # Lancer le serveur Flask dans un thread séparé
    flask_thread = Thread(target=start_flask)
    flask_thread.start()

    # Exécuter le node ROS2
    rclpy.spin(sensor_subscriber)
