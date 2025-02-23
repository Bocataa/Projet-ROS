import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64
from flask import Flask, jsonify, request
from threading import Thread
from flask_cors import CORS
from std_srvs.srv import Trigger

app = Flask(__name__)
CORS(app) # pour eviter les erreurs de sécurité sur les routes

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
        
        # Client pour contrôler le servo copié/collé bp1.py
        self.button_state = 0
        self.cli = self.create_client(Trigger, 'set_servo_state')
        
        while not self.cli.wait_for_service(timeout_sec=1.0): # si le service est indispo apres un certaint timeout
            self.get_logger().warn('Service Servo non disponible, attente...')

    # callback pour les différents capteurs, copié/collé fichiers spécifiques
    def servo_callback(self, msg):
        global servo_angle
        servo_angle = msg.data  # Mise à jour de l'angle du servo
        self.get_logger().info(f"Nouvel angle du servo : {servo_angle}°")

    def temp_callback(self, msg):
        global temperature_value
        temperature_value = msg.data

    # copié/collé bp1.py le bouton/servo fonctionnent en client/serveur
    def send_request(self):
        # Envoi d'une requête au serveur du servo moteur
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        # Réception de la réponse du serveur
        try:
            response = future.result()
            self.get_logger().info(f"Réponse du servo : {response.message}")
        except Exception:
            self.get_logger().error(f"Échec de la requête : {str(Exception)}")


# Routes pour requêtes, taper adresse:5000/requete (ex localhost:5000/temp)
@app.route('/temp') # route Température
def get_data():
    return jsonify({'temperature': temperature_value}) #On transforme nla donnée en json pour la lire facilement dans le js

@app.route('/servo') # route servo
def get_servo():
    return jsonify({'servo_angle': servo_angle})

@app.route('/control_button', methods=['POST']) # On utilise la methode POST pour la requete (ne se voit pas dans l'url)
def virtual_button():
    node.send_request()  # Appel du service pour modifier l'angle du servo (pareil que bp1)
# L'ppui sur le bouton donne un statut d'erreur 500 (serveur) mais pas grave ça marche
# définition du thread pour faire tourner le node
def ros_node_thread():
    global node
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    
def main():
	# On sépare dans des thread différent le serveur flask et la node sinon problème car les nodes peuvent être blocantes 
    	# voir https://docs.python.org/fr/3.8/library/threading.html

    ros_thread = Thread(target=ros_node_thread, daemon=True) 
    ros_thread.start()
    app.run(host='0.0.0.0', port=5000)
	
if __name__ == '__main__':
	main()
    
