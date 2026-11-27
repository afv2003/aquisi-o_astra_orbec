"""Manobra de exemplo: seguir o carro da frente (ou parar atrás de um obstáculo) na faixa.

É uma manobra mínima, para mostrar a cadeia completa a funcionar. As manobras da parte 2 são do grupo.

Longitudinal (controlo de cruzeiro adaptativo, intervalo de tempo constante):
    distância desejada  d_des = distancia_minima + tempo_seguranca * v
    velocidade pedida   v_cmd = v_frente + ganho_distancia * (distância - d_des), limitada a [0, v_max]
    v_frente estima-se pela variação da distância: v_frente = v + d(distância)/dt (filtrada)
Lateral: pure pursuit sobre /estrada/centro (aula teórica 11), com look-ahead proporcional à velocidade.

Subscreve  distancia_livre  std_msgs/Float32    (de faixa_livre)
           /estrada/centro  nav_msgs/Path       (no base_link)
           /odom            nav_msgs/Odometry   (velocidade atual)
Publica    /cmd_vel         geometry_msgs/Twist
           estado_manobra   std_msgs/String     CRUZEIRO | SEGUIR | PARADO | SEM DADOS
Sem distância ou sem estrada recentes, o carro para: na dúvida, não se avança.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String


class Seguir(Node):

    def __init__(self):
        super().__init__('seguir')
        for nome, valor in (('v_max', 10.0), ('tempo_seguranca', 1.8), ('distancia_minima', 5.0),
                            ('ganho_distancia', 0.3), ('alcance', 40.0), ('x_frente', 3.0),
                            ('lookahead_min', 5.0), ('lookahead_ganho', 0.8), ('frequencia', 20.0),
                            ('timeout', 0.5)):
            self.declare_parameter(nome, valor)
        self.dist = self.t_dist = None
        self.taxa = 0.0                       # d(distância)/dt filtrada
        self.livre_antes = True
        self.estrada = self.t_estrada = None
        self.v = 0.0
        self.create_subscription(Float32, 'distancia_livre', self.on_dist, 10)
        self.create_subscription(Path, '/estrada/centro', self.on_estrada, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_estado = self.create_publisher(String, 'estado_manobra', 10)
        self.create_timer(1.0 / self.get_parameter('frequencia').value, self.ciclo)

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_dist(self, msg):
        t = self.agora()
        livre = msg.data >= self.get_parameter('alcance').value - self.get_parameter('x_frente').value - 0.5
        if self.dist is not None and not livre and not self.livre_antes and t > self.t_dist:
            bruta = (msg.data - self.dist) / (t - self.t_dist)
            self.taxa += 0.25 * (float(np.clip(bruta, -15, 15)) - self.taxa)
        elif livre or self.dist is None:
            self.taxa = 0.0
        self.dist, self.t_dist, self.livre_antes = msg.data, t, livre

    def on_estrada(self, msg):
        self.estrada = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        self.t_estrada = self.agora()

    def on_odom(self, msg):
        self.v = msg.twist.twist.linear.x

    def ciclo(self):
        gp = lambda n: self.get_parameter(n).value
        t = self.agora()
        cmd = Twist()
        ok = (self.dist is not None and t - self.t_dist < gp('timeout')
              and self.estrada is not None and len(self.estrada) > 1 and t - self.t_estrada < gp('timeout'))
        if not ok:
            self.publicar(cmd, 'SEM DADOS')
            return

        # ---- longitudinal
        v = self.v
        if self.livre_antes:
            v_cmd, estado = gp('v_max'), 'CRUZEIRO'
        else:
            d_des = gp('distancia_minima') + gp('tempo_seguranca') * v
            v_frente = max(0.0, v + self.taxa)
            v_cmd = float(np.clip(v_frente + gp('ganho_distancia') * (self.dist - d_des), 0.0, gp('v_max')))
            if self.dist < gp('distancia_minima'):
                v_cmd = 0.0
            estado = 'PARADO' if v < 0.5 and self.dist < gp('distancia_minima') + 2.0 else 'SEGUIR'

        # ---- lateral: pure pursuit sobre a linha de centro (base_link no eixo traseiro)
        p = self.estrada
        ld = max(gp('lookahead_min'), gp('lookahead_ganho') * v)
        dist0 = np.hypot(p[:, 0], p[:, 1])
        i0 = int(np.argmin(dist0))
        frente = p[i0:]
        arco = np.r_[0.0, np.cumsum(np.hypot(*np.diff(frente, axis=0).T))]
        k = int(np.searchsorted(arco, ld))
        gx, gy = frente[min(k, len(frente) - 1)]
        alpha = math.atan2(gy, gx)
        curvatura = 2.0 * math.sin(alpha) / max(math.hypot(gx, gy), 1e-3)
        cmd.linear.x = v_cmd
        cmd.angular.z = max(v, 0.5) * curvatura if v_cmd > 0 else 0.0
        self.publicar(cmd, estado)

    def publicar(self, cmd, estado):
        self.pub_cmd.publish(cmd)
        self.pub_estado.publish(String(data=estado))
        self.get_logger().info(f'{estado:9s} v = {self.v:4.1f} m/s  pedida = {cmd.linear.x:4.1f}  '
                               f'distância = {self.dist if self.dist is not None else float("nan"):5.1f} m',
                               throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = Seguir()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
