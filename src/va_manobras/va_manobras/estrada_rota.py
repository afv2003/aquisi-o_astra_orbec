"""Fonte da estrada por "rota gravada": publica a linha de centro da faixa no referencial do carro.

Ideia (teach and repeat): conduz-se o percurso uma vez, grava-se a trajetória (GNSS ou odometria)
e essa trajetória passa a ser o centro da faixa. Em cada ciclo, este nó vê onde está o carro na
rota (TF frame_rota -> base_link) e publica o troço à frente no base_link.

Contrato da estrada (comum a todas as fontes: rota gravada, linhas da câmara, bermas do LiDAR, simulador)
  /estrada/centro   nav_msgs/Path   linha de centro da faixa do carro, no base_link, do fundo para a frente
  largura da faixa: parâmetro largura_faixa de quem consome (3,5 m por omissão)
Também publica /estrada/rota (a rota completa no frame_rota, 1 Hz) para visualização.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .estrada import Estrada, pista_oval


def path_msg(x, y, header):
    msg = Path()
    msg.header = header
    for xi, yi in zip(x, y):
        p = PoseStamped()
        p.header = header
        p.pose.position.x, p.pose.position.y = float(xi), float(yi)
        p.pose.orientation.w = 1.0
        msg.poses.append(p)
    return msg


class EstradaRota(Node):

    def __init__(self):
        super().__init__('estrada_rota')
        for nome, valor in (('rota', ''), ('frame_rota', 'odom'), ('frame_base', 'base_link'),
                            ('frequencia', 10.0), ('atras', 5.0), ('frente', 50.0), ('passo', 1.0)):
            self.declare_parameter(nome, valor)
        rota = self.get_parameter('rota').value
        self.estrada = Estrada.de_csv(rota) if rota else pista_oval()
        self.get_logger().info(f'rota {rota or "(pista oval de teste)"}: {self.estrada.comprimento:.0f} m')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Path, '/estrada/centro', 10)
        self.pub_rota = self.create_publisher(Path, '/estrada/rota', 10)
        self.create_timer(1.0 / self.get_parameter('frequencia').value, self.ciclo)
        self.create_timer(1.0, self.publicar_rota)

    def ciclo(self):
        gp = lambda n: self.get_parameter(n).value
        try:
            tf = self.tf_buffer.lookup_transform(gp('frame_rota'), gp('frame_base'), Time())
        except TransformException as e:
            self.get_logger().warn(f'sem a pose do carro na rota: {e}', throttle_duration_sec=5.0)
            return
        cx, cy = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        s_carro = float(self.estrada.para_frenet(cx, cy)[0][0])
        x, y = self.estrada.trecho(s_carro - gp('atras'), s_carro + gp('frente'), gp('passo'))
        # do frame da rota para o base_link
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x - cx, y - cy
        xb, yb = c * dx + s * dy, -s * dx + c * dy
        header = tf.header
        header.frame_id = gp('frame_base')
        self.pub.publish(path_msg(xb, yb, header))

    def publicar_rota(self):
        x, y = self.estrada.trecho(0.0, self.estrada.comprimento, 2.0)
        header = Path().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('frame_rota').value
        self.pub_rota.publish(path_msg(np.asarray(x), np.asarray(y), header))


def main():
    rclpy.init()
    node = EstradaRota()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
