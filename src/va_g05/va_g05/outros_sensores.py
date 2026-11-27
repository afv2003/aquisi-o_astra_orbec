# -*- coding: utf-8 -*-
"""
outros_sensores.py - Tópicos de outros sensores do carro (de outros grupos) que o G5 grava e usa (parte 2).

Na parte 2, os outros grupos publicam na mesma rede (mesmo ROS_DOMAIN_ID, mesmo relógio por NTP), entre
outros, o GNSS, a odometria, o TF do carro e a estrada (contrato da aula 11). O G5:
  * grava-os no bag da sessão, nos trechos, junto com as câmaras (gravacao_online.py), se existirem;
  * volta a publicá-los na reprodução de um bag (reprodutor.py), para os outros nós os receberem como ao vivo;
  * usa a estrada (/estrada/centro) nos avisos (decisao.py, coordenadas de Frenet) e o GNSS no mapa da
    janela de resultados do offline (tab Percurso).
Sem esses tópicos, tudo funciona como na parte 1.

Tópicos por omissão (variável OUTROS_SENSORES, no .env ou antes do scripts/online.sh: lista separada por
espaços; "topico=pacote/msg/Tipo" para um tópico com outro nome; "nenhum" desliga):
  /fix             sensor_msgs/msg/NavSatFix    GNSS (latitude, longitude)
  /odom            nav_msgs/msg/Odometry        odometria (pose e velocidade do carro)
  /tf              tf2_msgs/msg/TFMessage       TF do carro (odom -> base_link, map -> odom, ...)
  /estrada/centro  nav_msgs/msg/Path            linha de centro da faixa do carro, no base_link (aula 11)
"""
import importlib
import os

from . import gravacao_online

_SEP = "=" * 80 + "\n"
_HEADER = ("MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n" + _SEP
           + "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n")
_POSE = ("MSG: geometry_msgs/Pose\ngeometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\n" + _SEP
         + "MSG: geometry_msgs/Point\nfloat64 x\nfloat64 y\nfloat64 z\n" + _SEP
         + "MSG: geometry_msgs/Quaternion\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n")
DEFINICAO_NAVSATFIX = ("std_msgs/Header header\nsensor_msgs/NavSatStatus status\nfloat64 latitude\nfloat64 longitude\n"
                       "float64 altitude\nfloat64[9] position_covariance\nuint8 position_covariance_type\n" + _SEP
                       + _HEADER + _SEP + "MSG: sensor_msgs/NavSatStatus\nint8 status\nuint16 service\n")
DEFINICAO_ODOMETRIA = ("std_msgs/Header header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\n"
                       "geometry_msgs/TwistWithCovariance twist\n" + _SEP + _HEADER + _SEP
                       + "MSG: geometry_msgs/PoseWithCovariance\ngeometry_msgs/Pose pose\nfloat64[36] covariance\n"
                       + _SEP + _POSE + _SEP
                       + "MSG: geometry_msgs/TwistWithCovariance\ngeometry_msgs/Twist twist\nfloat64[36] covariance\n"
                       + _SEP + "MSG: geometry_msgs/Twist\ngeometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular\n"
                       + _SEP + "MSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n")

# tipos que se sabem gravar (definição da mensagem para o MCAP)
DEFINICOES = {"sensor_msgs/msg/NavSatFix": DEFINICAO_NAVSATFIX,
              "nav_msgs/msg/Odometry": DEFINICAO_ODOMETRIA,
              "tf2_msgs/msg/TFMessage": gravacao_online.DEFINICAO_TF,
              "nav_msgs/msg/Path": gravacao_online.DEFINICAO_CAMINHO}
TOPICO_ESTRADA = "/estrada/centro"
TIPO_GNSS = "sensor_msgs/msg/NavSatFix"
POR_OMISSAO = {"/fix": TIPO_GNSS, "/odom": "nav_msgs/msg/Odometry", "/tf": "tf2_msgs/msg/TFMessage",
               TOPICO_ESTRADA: "nav_msgs/msg/Path"}


def configurados(texto=None):
    """{tópico: tipo} a partir da variável OUTROS_SENSORES (vazia: os por omissão)."""
    texto = (os.environ.get("OUTROS_SENSORES", "") if texto is None else texto).strip()
    if not texto:
        return dict(POR_OMISSAO)
    if texto.lower() in ("nenhum", "0", "nao", "não"):
        return {}
    res = {}
    for item in texto.replace(",", " ").split():
        topico, _, tipo = item.partition("=")
        tipo = tipo or POR_OMISSAO.get(topico)
        if tipo in DEFINICOES:
            res[topico] = tipo
        else:
            print(f"Aviso: OUTROS_SENSORES: {item} ignorado (tipo desconhecido; use topico=" +
                  "|".join(DEFINICOES) + ").")
    return res


def chave(topico):
    """Chave do canal no GravadorTrechos."""
    return f"outro:{topico}"


def canais_para_gravar(topicos):
    """{chave: (tópico, tipo, definição)} para o GravadorTrechos (extra)."""
    return {chave(t): (t, tipo, DEFINICOES[tipo]) for t, tipo in topicos.items()}


def classe(tipo):
    """Classe ROS de "pacote/msg/Tipo" (ImportError se o pacote não existir)."""
    pacote, _, nome = tipo.split("/")
    return getattr(importlib.import_module(f"{pacote}.msg"), nome)


def stamp_ns(header, alternativa=None):
    """header.stamp em ns (ou a alternativa, se o stamp for zero)."""
    t = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
    return t if t > 0 or alternativa is None else int(alternativa)


def pontos_do_caminho(msg):
    """nav_msgs/Path -> (xs, ys) (listas)."""
    return [p.pose.position.x for p in msg.poses], [p.pose.position.y for p in msg.poses]
