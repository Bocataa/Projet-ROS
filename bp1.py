import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
import termios
import tty
import select

class BP1Client(Node):

    def __init__(self):
        super().__init__('bp1_client')
        
        # Création du client ROS2 pour contacter le serveur du servo
        self.cli = self.create_client(Trigger, 'set_servo_state')

        # Attente de la disponibilité du service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service Servo non disponible, attente...')

        # Initialisation du mode simulation clavier
        self.button_state = False  # État initial du bouton

    def send_request(self):
        """Envoi d'une requête au serveur du servo moteur"""
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Réception de la réponse du serveur"""
        try:
            response = future.result()
            self.get_logger().info(f"Réponse du servo : {response.message}")
        except Exception as e:
            self.get_logger().error(f"Échec de la requête : {str(e)}")

	# Fonction pompée sur internet pour récupéré la presion touche clavier
    def keyboard_listener(self):
        # Écoute le clavier pour détecter la touche Espace
        self.get_logger().info("Appuyez sur ESPACE pour simuler le bouton (CTRL+C pour quitter)")

        while rclpy.ok():
            key = self.get_key()
            if key == " ":
                self.button_state = not self.button_state
                self.get_logger().info(f"Bouton simulé : {self.button_state}")
                self.send_request()

    def get_key(self):
        # Récupère une touche du clavier sans bloquer le terminal
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

def main(args=None):
    rclpy.init(args=args)
    bp1_client = BP1Client()

    try:
        bp1_client.keyboard_listener()
    except KeyboardInterrupt:
        print("\nFermeture du simulateur.")
    
    bp1_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

