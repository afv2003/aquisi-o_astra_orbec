"""Simulador 2D de estrada para testar manobras sem o carro nem o Gazebo.

O carro do grupo (modelo de bicicleta, aula teórica 11) anda numa estrada de duas faixas definida
por uma rota em CSV (a mesma que o nó estrada_rota usa). Há outros veículos com guião:
  cenario 'seguir'  um carro à frente na nossa faixa, que acelera e trava de tempos a tempos
  cenario 'parado'  um carro parado na nossa faixa (é retirado depois de o nosso carro parar)
  cenario 'livre'   só a estrada
  vizinho: true     mais um carro na faixa da esquerda, mais lento (em curva parece estar à frente)

Recebe   /cmd_vel   geometry_msgs/Twist   linear.x = velocidade (m/s), angular.z = velocidade angular (rad/s)
Publica  /scan      sensor_msgs/LaserScan  LiDAR 2D simulado na frente do carro (frame lidar2d_frente)
         /odom      nav_msgs/Odometry      pose e velocidade do carro no frame odom
         /tf        odom -> base_link      /tf_static  base_link -> lidar2d_frente
         /sim/veiculos         visualization_msgs/MarkerArray  carros e bermas (para ver no Lichtblick)
         /sim/distancia_real   std_msgs/Float32  distância verdadeira ao carro da frente (para avaliar)
         /sim/desvio_lateral   std_msgs/Float32  d verdadeiro do carro em relação ao centro da faixa
O cenário só arranca quando chega o primeiro /cmd_vel. Mudar cenario ou vizinho com ros2 param set
recomeça o cenário do início.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .estrada import Estrada, pista_oval

EIXOS = 2.55            # distância entre eixos do ATLASCAR2 (Mitsubishi i-MiEV), m
FRENTE = 3.0            # do eixo traseiro (base_link) à frente do carro, m
TRAS = 0.45             # do eixo traseiro à traseira, m
LARGURA = 1.5
DELTA_MAX = 0.6         # ângulo de direção máximo, rad
COMP_OUTRO, LARG_OUTRO = 4.5, 1.8


def quat_yaw(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def retangulo(cx, cy, yaw, comp, larg):
    c, s = math.cos(yaw), math.sin(yaw)
    cantos = [(comp / 2, larg / 2), (-comp / 2, larg / 2), (-comp / 2, -larg / 2), (comp / 2, -larg / 2)]
    p = [(cx + c * a - s * b, cy + s * a + c * b) for a, b in cantos]
    return [(p[i][0], p[i][1], p[(i + 1) % 4][0], p[(i + 1) % 4][1]) for i in range(4)]


class Outro:
    """Veículo com guião: anda ao longo da estrada a uma distância lateral fixa."""

    def __init__(self, s, d, v, plano=None, ciclo=0.0):
        self.s, self.d, self.v = s, d, v
        self.plano = plano or [(0.0, v)]      # [(instante, velocidade alvo), ...]
        self.ciclo = ciclo
        self.visivel = True

    def passo(self, t, dt):
        tc = t % self.ciclo if self.ciclo else t
        alvo = [v for t0, v in self.plano if t0 <= tc][-1]
        a = np.clip((alvo - self.v) / 1.0, -3.0, 1.5)
        self.v = max(0.0, self.v + a * dt)
        self.s += self.v * dt


class SimEstrada(Node):

    def __init__(self):
        super().__init__('sim_estrada')
        for nome, valor in (('rota', ''), ('cenario', 'seguir'), ('vizinho', True),
                            ('largura_faixa', 3.5), ('frame_odom', 'odom'), ('frame_base', 'base_link'),
                            ('frame_sensor', 'lidar2d_frente'), ('sensor_x', 3.0),
                            ('frequencia', 50.0), ('frequencia_scan', 10.0), ('ruido', 0.02),
                            ('alcance_scan', 40.0)):
            self.declare_parameter(nome, valor)
        gp = lambda n: self.get_parameter(n).value
        rota = gp('rota')
        self.estrada = Estrada.de_csv(rota) if rota else pista_oval()
        self.get_logger().info(f'estrada: {self.estrada.comprimento:.0f} m '
                               f'({"fechada" if self.estrada.fechada else "aberta"}), cenário {gp("cenario")}')
        w = gp('largura_faixa')
        self.largura = w
        # o carro começa no início da rota, no centro da faixa, alinhado com a estrada
        self.montar_cenario()

        # bermas: lancis 0,5 m para fora das duas faixas, como segmentos para o LiDAR
        sb = np.arange(0.0, self.estrada.comprimento + (0 if self.estrada.fechada else 1e-9), 2.0)
        self.bermas = []
        for d in (-w / 2 - 0.5, 1.5 * w + 0.5):
            bx, by = self.estrada.de_frenet(sb, d)
            if self.estrada.fechada:
                bx, by = np.r_[bx, bx[:1]], np.r_[by, by[:1]]
            self.bermas.append((bx, by))
        self.seg_bermas = np.vstack([np.column_stack([bx[:-1], by[:-1], bx[1:], by[1:]])
                                     for bx, by in self.bermas])

        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/sim/veiculos', 10)
        self.pub_gap = self.create_publisher(Float32, '/sim/distancia_real', 10)
        self.pub_d = self.create_publisher(Float32, '/sim/desvio_lateral', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = gp('frame_base')
        st.child_frame_id = gp('frame_sensor')
        st.transform.translation.x = gp('sensor_x')
        st.transform.translation.z = 0.4
        st.transform.rotation.w = 1.0
        self.tf_static.sendTransform(st)
        self.rng = np.random.default_rng(1)
        self.dt = 1.0 / gp('frequencia')
        self.create_timer(self.dt, self.passo)
        self.create_timer(1.0 / gp('frequencia_scan'), self.publicar_scan)
        self.create_timer(1.0, self.publicar_bermas)

    def montar_cenario(self):
        """(Re)começa o cenário: carro no início da rota, outros veículos conforme o parâmetro cenario."""
        gp = lambda n: self.get_parameter(n).value
        x0, y0 = self.estrada.de_frenet(0.0, 0.0)
        self.x, self.y = float(x0[0]), float(y0[0])
        self.th = float(self.estrada.rumo(0.0)[0])
        self.v, self.v_ref, self.w_ref, self.w = 0.0, 0.0, 0.0, 0.0
        self.t, self.iniciado = 0.0, False
        self.parado_desde, self.s_retirado = None, None
        self.cenario, self.com_vizinho = gp('cenario'), gp('vizinho')
        self.outros = {}
        if self.cenario == 'seguir':
            self.outros['frente'] = Outro(28.0, 0.0, 0.0, [(0, 9.0), (14, 3.0), (24, 11.0), (36, 8.0)], ciclo=48.0)
        elif self.cenario == 'parado':
            self.outros['parado'] = Outro(min(150.0, 0.6 * self.estrada.comprimento), 0.0, 0.0, [(0, 0.0)])
        if self.com_vizinho:
            self.outros['vizinho'] = Outro(60.0, self.largura, 6.0, [(0, 6.0)])

    # ------------------------------------------------------------------ dinâmica
    def on_cmd(self, msg):
        if not self.iniciado:
            self.get_logger().info('primeiro /cmd_vel recebido: o cenário arranca')
        self.iniciado = True
        self.v_ref, self.w_ref = float(msg.linear.x), float(msg.angular.z)
        self.ultimo_cmd = self.get_clock().now()

    def passo(self):
        dt = self.dt
        if (self.get_parameter('cenario').value, self.get_parameter('vizinho').value) != (self.cenario, self.com_vizinho):
            self.get_logger().info(f'novo cenário: {self.get_parameter("cenario").value} (recomeça do início)')
            self.montar_cenario()
        if self.iniciado:
            if (self.get_clock().now() - self.ultimo_cmd).nanoseconds > 0.5e9:
                self.v_ref, self.w_ref = 0.0, 0.0          # sem comandos: o carro para
            self.t += dt
            a = np.clip((self.v_ref - self.v) / 0.5, -6.0, 2.0)   # atraso de 0,5 s e limites de aceleração
            self.v = max(0.0, self.v + a * dt)
            w_max = self.v * math.tan(DELTA_MAX) / EIXOS
            self.w = float(np.clip(self.w_ref, -w_max, w_max))
            self.th += self.w * dt
            self.x += self.v * math.cos(self.th) * dt
            self.y += self.v * math.sin(self.th) * dt
            for o in self.outros.values():
                o.passo(self.t, dt)
            self.nao_atravessar()
            self.cenario_parado()
        self.publicar_odom()

    def nao_atravessar(self):
        """Um carro da nossa faixa que nos apanhe por trás (numa pista fechada) fica atrás de nós."""
        s_ego = self.s_ego()
        for o in self.outros.values():
            if abs(o.d) < self.largura / 2 and o.v > self.v:
                atras = self.distancia_ao_longo(o.s + COMP_OUTRO / 2, s_ego - TRAS)
                if atras < 8.0:
                    o.v = self.v

    def cenario_parado(self):
        o = self.outros.get('parado')
        if o is None:
            return
        s_ego = self.s_ego()
        if o.visivel and self.v < 0.2:
            self.parado_desde = self.parado_desde or self.t
            if self.t - self.parado_desde > 4.0:
                o.visivel, self.s_retirado = False, s_ego
                self.get_logger().info('carro parado retirado da estrada')
        else:
            self.parado_desde = None
        if not o.visivel and self.distancia_ao_longo(self.s_retirado, s_ego) > 120.0:
            o.visivel = True

    def s_ego(self):
        return float(self.estrada.para_frenet(self.x, self.y)[0][0])

    def distancia_ao_longo(self, s_de, s_para):
        ds = s_para - s_de
        if self.estrada.fechada:
            ds = ds % self.estrada.comprimento
        return ds

    def pose_outro(self, o):
        x, y = self.estrada.de_frenet(o.s, o.d)
        return float(x[0]), float(y[0]), float(self.estrada.rumo(o.s)[0])

    # ------------------------------------------------------------------ publicação
    def publicar_odom(self):
        now = self.get_clock().now().to_msg()
        qz, qw = quat_yaw(self.th)
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.get_parameter('frame_odom').value
        tf.child_frame_id = self.get_parameter('frame_base').value
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)
        self.publicar_veiculos(now)
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = tf.header.frame_id
        od.child_frame_id = tf.child_frame_id
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.pub_odom.publish(od)
        s_ego, d_ego = self.estrada.para_frenet(self.x, self.y)
        self.pub_d.publish(Float32(data=float(d_ego[0])))
        o = self.outros.get('frente') or self.outros.get('parado')
        if o is not None and o.visivel:
            gap = self.distancia_ao_longo(float(s_ego[0]) + FRENTE, o.s - COMP_OUTRO / 2)
            self.pub_gap.publish(Float32(data=float(gap)))

    def publicar_scan(self):
        gp = lambda n: self.get_parameter(n).value
        sx = gp('sensor_x')
        c, s = math.cos(self.th), math.sin(self.th)
        ox, oy = self.x + sx * c, self.y + sx * s                 # sensor no mundo
        alcance = gp('alcance_scan')
        segs = [self.seg_bermas]
        for o in self.outros.values():
            if o.visivel:
                segs.append(np.array(retangulo(*self.pose_outro(o), COMP_OUTRO, LARG_OUTRO)))
        seg = np.vstack(segs)
        # só os segmentos perto do sensor, e no referencial do sensor
        perto = np.minimum(np.hypot(seg[:, 0] - ox, seg[:, 1] - oy),
                           np.hypot(seg[:, 2] - ox, seg[:, 3] - oy)) < alcance + 5.0
        seg = seg[perto]
        ax, ay = seg[:, 0] - ox, seg[:, 1] - oy
        bx, by = seg[:, 2] - ox, seg[:, 3] - oy
        ax, ay = c * ax + s * ay, -s * ax + c * ay
        bx, by = c * bx + s * by, -s * bx + c * by

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = gp('frame_sensor')
        scan.angle_min, scan.angle_max = -math.pi / 2, math.pi / 2
        n = 361
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (n - 1)
        scan.range_min, scan.range_max = 0.1, alcance
        ang = scan.angle_min + scan.angle_increment * np.arange(n)
        dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
        ex, ey = (bx - ax)[None, :], (by - ay)[None, :]
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            dist = (ax * ey - ay * ex) / den                      # ao longo do raio
            u = (ax[None, :] * dy - ay[None, :] * dx) / den       # ao longo do segmento
        ok = (np.abs(den) > 1e-9) & (dist > 0) & (u >= 0) & (u <= 1)
        r = np.where(ok, dist, np.inf).min(axis=1) if seg.size else np.full(n, np.inf)
        r = r + self.rng.normal(0.0, gp('ruido'), n)
        r[~np.isfinite(r) | (r > alcance)] = np.inf
        scan.ranges = r.astype(float).tolist()
        self.pub_scan.publish(scan)

    def publicar_veiculos(self, now):
        """Caixas dos carros, ao ritmo da física e com o mesmo instante do /tf (senão "saltam" na visualização)."""
        arr = MarkerArray()

        def caixa(i, frame, x, y, yaw, comp, larg, cor):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, now
            m.ns, m.id, m.type, m.action = 'veiculos', i, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.75
            m.pose.orientation.z, m.pose.orientation.w = quat_yaw(yaw)
            m.scale.x, m.scale.y, m.scale.z = comp, larg, 1.5
            m.color.r, m.color.g, m.color.b, m.color.a = (*cor, 0.85)
            return m

        # o nosso carro no base_link (acompanha a TF exatamente); os outros no frame odom
        ego = caixa(0, self.get_parameter('frame_base').value, (FRENTE - TRAS) / 2, 0.0, 0.0,
                    FRENTE + TRAS, LARGURA, (0.57, 0.83, 0.0))
        ego.frame_locked = True
        arr.markers.append(ego)
        frame = self.get_parameter('frame_odom').value
        for i, (nome, o) in enumerate(self.outros.items(), start=1):
            m = caixa(i, frame, *self.pose_outro(o), COMP_OUTRO, LARG_OUTRO,
                      (0.73, 0.11, 0.11) if nome != 'vizinho' else (0.11, 0.31, 0.85))
            m.action = Marker.ADD if o.visivel else Marker.DELETE
            arr.markers.append(m)
        self.pub_mk.publish(arr)

    def publicar_bermas(self):
        """Bermas (fixas): 1 vez por segundo chega; os marcadores ficam até serem substituídos."""
        frame = self.get_parameter('frame_odom').value
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for k, (bx, by) in enumerate(self.bermas):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, now
            m.ns, m.id, m.type, m.action = 'bermas', k, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.color.r = m.color.g = m.color.b = 0.35
            m.color.a = 1.0
            m.points = [Point(x=float(a), y=float(b)) for a, b in zip(bx, by)]
            arr.markers.append(m)
        self.pub_mk.publish(arr)


def main():
    rclpy.init()
    node = SimEstrada()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
