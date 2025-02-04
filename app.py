import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from flask import Flask, jsonify, request
from threading import Thread

from flask_cors import CORS # permet d'éviter les erreurs entre navigateur/serveur 

app = Flask(__name__)
CORS(app)  # Active CORS pour toutes les routes


# Variables globales pour stocker les valeurs des capteurs
temperature_value = 0.0
led_state = False
bp1_state = False
servo_angle = 50


class SensorNode(Node):

    def __init__(self):
        super().__init__('control_node')
        # Souscriptions aux topics ROS2 pour le contrôle
        self.create_subscription(Bool, 'bp1', self.bp_callback, 10)
        self.create_subscription(Float64, 'capt_temp', self.temp_callback, 10)

    def bp_callback(self, msg):
        global bp1_state
        bp1_state = msg.data  #lit la valeur de bp1

    def temp_callback(self, msg):
        global temperature_value
        temperature_value = msg.data  # Mettre à jour la température



@app.route('/temp')
def get_temp():
    # Route pour récupérer les données des capteurs
    return jsonify({'temperature': temperature_value})

@app.route('/servoAngle')
def get_servo():
    return jsonify({'servoAngle': servo_angle})
    


def ros_thread():
    # Démarrer les nodes ROS2 dans un thread séparé (l'exécution des nodes est bloquant)
    rclpy.init()
    sensor_node = SensorNode()
    rclpy.spin(sensor_node)


if __name__ == '__main__':

    # Démarrer le serveur Flask et les nodes ROS2 en parallèle
    Thread(target=ros_thread).start()
    app.run(host='0.0.0.0', port=5000)

