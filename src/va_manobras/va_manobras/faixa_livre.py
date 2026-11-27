"""Ocupação da faixa: grelha local + estrada -> distância livre na faixa do carro.

Junta as duas peças que a parte 2 precisa: o que os sensores veem (a grelha local de qualquer grupo)
e onde está a estrada (/estrada/centro). Cada célula ocupada é projetada em Frenet (s, d) e conta
como obstáculo se estiver dentro da faixa (|d| < largura_faixa / 2).

Subscreve  grelha_local     nav_msgs/OccupancyGrid  (do grupo, no base_link)
           /estrada/centro  nav_msgs/Path           (no base_link)
Publica    distancia_livre  std_msgs/Float32        da frente do carro ao primeiro obstáculo na faixa (m);
                                                    igual a alcance - x_frente se a faixa está livre
           obstaculo        geometry_msgs/PointStamped  ponto que limita a distância (base_link)
           faixa            geometry_msgs/PolygonStamped contorno da faixa considerada (para ver)
Parâmetro modo: 'frenet' (usa a estrada) ou 'reto' (finge que a estrada é reta: para comparar).
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32

from va_planeamento.grid import LocalGrid

from .estrada import Estrada
from .faixa import corredor, ocupacao_da_faixa, pontos_ocupados


class FaixaLivre(Node):

    def __init__(self):
        super().__init__('faixa_livre')
        for nome, valor in (('largura_faixa', 3.5), ('x_frente', 3.0), ('alcance', 40.0),
                            ('modo', 'frenet'), ('idade_max_estrada', 1.0)):
            self.declare_parameter(nome, valor)
        self.estrada_msg = None
        self.create_subscription(OccupancyGrid, 'grelha_local', self.on_grelha, 10)
        self.create_subscription(Path, '/estrada/centro', self.on_estrada, 10)
        self.pub_dist = self.create_publisher(Float32, 'distancia_livre', 10)
        self.pub_obs = self.create_publisher(PointStamped, 'obstaculo', 10)
        self.pub_faixa = self.create_publisher(PolygonStamped, 'faixa', 10)

    def on_estrada(self, msg):
        self.estrada_msg = msg

    def on_grelha(self, msg):
        gp = lambda n: self.get_parameter(n).value
        modo = gp('modo')
        estrada = None
        if modo != 'reto':
            m = self.estrada_msg
            # estrada e grelha têm de ser do mesmo instante (compara os stamps: funciona também com bags)
            idade = abs((Time.from_msg(msg.header.stamp) - Time.from_msg(m.header.stamp)).nanoseconds) * 1e-9 \
                if m else None
            if m is None or len(m.poses) < 2 or idade > gp('idade_max_estrada'):
                # sem estrada não se sabe o que é "a minha faixa": não se publica nada
                self.get_logger().warn('sem /estrada/centro recente: distancia_livre não publicada',
                                       throttle_duration_sec=5.0)
                return
            estrada = Estrada([p.pose.position.x for p in m.poses], [p.pose.position.y for p in m.poses],
                              fechada=False)
        g = LocalGrid.de_msg(msg)
        x, y = pontos_ocupados(g)
        meia, xf, alc = gp('largura_faixa') / 2, gp('x_frente'), gp('alcance')
        oc = ocupacao_da_faixa(x, y, estrada, meia, xf, alc, modo)

        self.pub_dist.publish(Float32(data=float(oc.distancia)))
        if oc.ponto is not None:
            p = PointStamped()
            p.header = msg.header
            p.point.x, p.point.y = oc.ponto
            self.pub_obs.publish(p)
        poly = PolygonStamped()
        poly.header = msg.header
        poly.polygon.points = [Point32(x=float(a), y=float(b))
                               for a, b in corredor(estrada, meia, oc.s_carro, xf, xf + oc.distancia, 1.0, modo)]
        self.pub_faixa.publish(poly)
        self.get_logger().info(
            f'{"LIVRE" if oc.livre else "OCUPADA"}  distância livre na faixa = {oc.distancia:5.1f} m  (modo {modo})',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = FaixaLivre()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
