#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool, String


class DemoPublicadores(Node):

    def __init__(self):
        super().__init__('demo_publicadores')

        # Publicadores
        self.pub_entero = self.create_publisher(
            Int32, '/demo/entero', 10
        )

        self.pub_booleano = self.create_publisher(
            Bool, '/demo/booleano', 10
        )

        self.pub_color = self.create_publisher(
            String, '/demo/color', 10
        )

        # Valores iniciales
        self.numero = 0
        self.estado = True

        self.colores = [
            'rojo',
            'verde',
            'azul',
            'amarillo',
            'naranja'
        ]
        self.indice_color = 0

        # Temporizadores independientes
        self.create_timer(2.0, self.publicar_entero)
        self.create_timer(5.0, self.publicar_booleano)
        self.create_timer(4.0, self.publicar_color)

        self.get_logger().info(
            'Nodo de demostración iniciado con 3 publicadores'
        )

    def publicar_entero(self):
        msg = Int32()
        msg.data = self.numero

        self.pub_entero.publish(msg)
        self.get_logger().info(f'ENTERO: {msg.data}')

        # Secuencia circular 0-9
        self.numero = (self.numero + 1) % 10

    def publicar_booleano(self):
        msg = Bool()
        msg.data = self.estado

        self.pub_booleano.publish(msg)
        self.get_logger().info(f'BOOLEANO: {msg.data}')

        # Alternar True / False
        self.estado = not self.estado

    def publicar_color(self):
        msg = String()
        msg.data = self.colores[self.indice_color]

        self.pub_color.publish(msg)
        self.get_logger().info(f'COLOR: {msg.data}')

        # Pasar al siguiente color
        self.indice_color = (
            self.indice_color + 1
        ) % len(self.colores)


def main(args=None):
    rclpy.init(args=args)

    node = DemoPublicadores()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()