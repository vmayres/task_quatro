import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Nó de publisher para /cmd_vel iniciado.')
        # Subscriber for LaserScan
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def run(self):
        # Cria a mensagem Twist
        self.ir_para_frente = Twist(linear=Vector3(x=0.2, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        # Espera alguns segundos antes de começar
        time.sleep(5)

        # Publica a mensagem para ir para frente
        self.publisher_.publish(self.ir_para_frente)

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.laser is None:
                continue

            self.distancia_frente = min(self.laser[80:100])  # -10 to 10 degrees
            
            self.get_logger().info(f'distancia_frente: {self.distancia_frente}')

            if self.distancia_frente < 0.05:  # 5 cm
                self.publisher_.publish(self.parar)
                break  # Para o loop quando o robô parar


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()

    try:
        while rclpy.ok():
            # Chama a função run para enviar a mensagem
            node.run()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
