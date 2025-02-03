import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from flask import Flask, jsonify, request
from threading import Thread

app = Flask(__name__)

# Variables globales pour stocker les valeurs des capteurs
temperature_value = 0.0
humidity_value = 0.0
light_value = 0.0
led_state = False
servo_angle = 0


class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        # Publishers pour chaque capteur
        self.temp_pub = self.create_publisher(Float64, 'capt_temp', 10)
        self.humidity_pub = self.create_publisher(Float64, 'capt_humidity', 10)
        self.light_pub = self.create_publisher(Float64, 'capt_light', 10)
        # Timer pour publier les données toutes les secondes
        self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        global temperature_value, humidity_value, light_value
        # Simuler la mise à jour des capteurs
        temperature_value += 0.1
        humidity_value += 0.1
        light_value += 0.1

        # Publier les nouvelles valeurs des capteurs
        self.temp_pub.publish(Float64(data=temperature_value))
        self.humidity_pub.publish(Float64(data=humidity_value))
        self.light_pub.publish(Float64(data=light_value))


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        # Souscriptions aux topics ROS2 pour le contrôle
        self.create_subscription(Bool, 'bp1', self.bp_callback, 10)
        self.create_subscription(Float64, 'capt_temp', self.temp_callback, 10)
        self.create_subscription(Float64, 'capt_light', self.light_callback, 10)

    def bp_callback(self, msg):
        global led_state
        led_state = msg.data  # Mettre à jour l'état de la LED

    def temp_callback(self, msg):
        global temperature_value
        temperature_value = msg.data  # Mettre à jour la température

    def light_callback(self, msg):
        global light_value
        light_value = msg.data  # Mettre à jour la luminosité


@app.route('/data')
def get_data():
    # Route pour récupérer les données des capteurs
    return jsonify({'temperature': temperature_value, 'humidity': humidity_value, 'light': light_value})


@app.route('/led', methods=['POST'])
def control_led():
    global led_state
    led_state = request.json.get('state', led_state)  # Modifier l'état de la LED
    return jsonify({'led_state': led_state})


@app.route('/servo', methods=['POST'])
def control_servo():
    global servo_angle
    servo_angle = request.json.get('angle', servo_angle)  # Modifier l'angle du servo
    return jsonify({'servo_angle': servo_angle})


def ros_thread():
    # Démarrer les nodes ROS2 dans un thread séparé
    rclpy.init()
    sensor_node = SensorPublisher()
    control_node = ControlNode()
    rclpy.spin(sensor_node)


if __name__ == '__main__':
    # Démarrer le serveur Flask et les nodes ROS2 en parallèle
    Thread(target=ros_thread).start()
    app.run(host='0.0.0.0', port=5000)

