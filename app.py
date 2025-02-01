from flask import Flask, jsonify
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

# Définition des Nodes ROS2 pour lire les données des capteurs
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

@app.route('/')
def get_sensor_data():
    # Retourne les données des capteurs sous forme JSON
    return jsonify({
        'temperature': temperature,
        'button_state': button_state,
        'light_level': light_level
    })

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
