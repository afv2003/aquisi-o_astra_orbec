"""Objetivo do planeador local (A*, aula 10) ao longo da estrada, e não "em frente".

O local_astar procura um caminho até um objetivo local. Por omissão o objetivo está 25 m à frente do carro
em y = 0, o que só faz sentido numa reta: numa curva fica fora da estrada e o A* sai por fora.
Este nó publica como objetivo o ponto da linha de centro da faixa que está `distancia` metros à frente,
medidos ao longo da estrada, ou antes disso se a faixa estiver ocupada: o objetivo fica `margem` metros antes do
primeiro obstáculo na faixa (um objetivo em cima do carro da frente obrigaria o A* a contorná-lo).

Subscreve  /estrada/centro  nav_msgs/Path            (no base_link)
           distancia_livre  std_msgs/Float32         (de faixa_livre, opcional)
Publica    objetivo         geometry_msgs/PoseStamped (no base_link; o local_astar subscreve-o)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .estrada import Estrada


class ObjetivoEstrada(Node):

    def __init__(self):
        super().__init__('objetivo_estrada')
        for nome, valor in (('distancia', 25.0), ('margem', 4.0), ('x_frente', 3.0), ('distancia_min', 5.0)):
            self.declare_parameter(nome, valor)
        self.livre = None
        self.create_subscription(Path, '/estrada/centro', self.on_estrada, 10)
        self.create_subscription(Float32, 'distancia_livre', lambda m: setattr(self, 'livre', m.data), 10)
        self.pub = self.create_publisher(PoseStamped, 'objetivo', 10)

    def on_estrada(self, msg):
        if len(msg.poses) < 2:
            return
        e = Estrada([p.pose.position.x for p in msg.poses], [p.pose.position.y for p in msg.poses], fechada=False)
        gp = lambda n: self.get_parameter(n).value
        ds = gp('distancia')
        if self.livre is not None:                       # antes do primeiro obstáculo na faixa
            ds = min(ds, max(gp('distancia_min'), self.livre + gp('x_frente') - gp('margem')))
        s = float(e.para_frenet(0.0, 0.0)[0][0]) + ds
        x, y = e.de_frenet(s, 0.0)
        yaw = float(e.rumo(s)[0])
        p = PoseStamped()
        p.header = msg.header
        p.pose.position.x, p.pose.position.y = float(x[0]), float(y[0])
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub.publish(p)


def main():
    rclpy.init()
    node = ObjetivoEstrada()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
