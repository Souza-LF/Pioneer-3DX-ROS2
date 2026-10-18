#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class PS4Teleop(Node):
    def __init__(self):
        super().__init__('ps4_teleop')
        
        # Publicando no novo tópico solicitado: /teleop_cmd
        self.publisher_ = self.create_publisher(Twist, '/teleop_cmd', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('Controle PS4 (Setas) iniciado. Segure R1 para mover no /teleop_cmd.')

    def joy_callback(self, msg):
        twist = Twist()
        
        # Botão 5 é o R1 no seu controle. Só move se ele estiver pressionado
        if msg.buttons[5] == 1: 
            # msg.axes[7] é a seta Cima/Baixo. 
            # Multiplicamos por 0.5 para limitar a velocidade máxima em 0.5 m/s
            twist.linear.x = msg.axes[7] * 0.5
            
            # msg.axes[6] é a seta Esquerda/Direita.
            # Multiplicamos por 0.5 para limitar a velocidade de giro em 0.5 rad/s
            twist.angular.z = msg.axes[6] * 0.5
        else:
            # Se soltar o R1, envia zeros para garantir que o robô pare
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PS4Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
