# -*- coding: utf-8 -*-
"""
geometria.py - Posição das câmaras no carro e passagem da imagem para o chão (e vice-versa).

Lê o config/montagem_camaras.yaml (posição e orientação de cada câmara no referencial do carro,
base_link) e, com a calibração da lente (K), liga cada píxel da imagem a um ponto do chão:

  * pixel_para_chao(u, v): o raio que sai da câmara por esse píxel e onde toca no chão (z = 0);
  * chao_para_pixel(x, y): onde aparece na imagem um ponto do chão (x, y) do referencial do carro.

Assume o chão plano (z = 0 no base_link), o que é razoável numa estrada até algumas dezenas de
metros. Só os píxeis abaixo do horizonte tocam no chão.

Referenciais (convenções do ROS, REP-103 e REP-105):
  base_link        x para a frente, y para a esquerda, z para cima (origem no chão, eixo traseiro);
  camara<N>_link   o "corpo" da câmara: x ao longo do eixo ótico, y para a esquerda, z para cima;
  camara<N>        o referencial ótico (o frame_id das imagens do usb_cam): z ao longo do eixo
                   ótico, x para a direita na imagem, y para baixo na imagem.
São estes os TF estáticos que o processar_tempo_real.py publica (/tf_static).
"""
import math
from pathlib import Path

import numpy as np

from . import caminhos

CONFIG = caminhos.CONFIG / "montagem_camaras.yaml"

# ótico -> corpo: z_ótico = x_corpo, x_ótico = -y_corpo, y_ótico = -z_corpo (colunas: eixos óticos no corpo)
OTICO_EM_CORPO = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])


def carregar_montagem(caminho=None):
    """Dicionário do montagem_camaras.yaml, ou None se o ficheiro não existir."""
    import yaml
    caminho = Path(caminho) if caminho else CONFIG
    if not caminho.exists():
        return None
    with open(caminho, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["camaras"] = {str(k): v for k, v in (cfg.get("camaras") or {}).items()}
    return cfg


def rpy_para_matriz(roll, pitch, yaw):
    """Rotação ROS (eixos fixos: roll em x, depois pitch em y, depois yaw em z), em radianos."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), \
        math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def matriz_para_quaternion(r):
    """(x, y, z, w) de uma matriz de rotação."""
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    return float(x), float(y), float(z), float(w)


def orientacao(cfg_cam):
    """(roll, pitch, yaw) em radianos do corpo da câmara no base_link, a partir do yaml."""
    fora = math.radians(float(cfg_cam.get("rotacao_para_fora_graus", 0)))
    esquerda = str(cfg_cam.get("lado", "esquerda")).lower().startswith("esq")
    if str(cfg_cam.get("sentido", "tras")).lower().startswith("tr"):
        yaw = math.pi - fora if esquerda else math.pi + fora     # para trás, rodada para fora
    else:
        yaw = fora if esquerda else -fora                         # para a frente, rodada para fora
    pitch = math.radians(float(cfg_cam.get("inclinacao_baixo_graus", 0)))   # positivo = para baixo
    roll = math.radians(float(cfg_cam.get("rotacao_eixo_graus", 0)))
    return roll, pitch, math.atan2(math.sin(yaw), math.cos(yaw))


def matriz_intrinseca(calibracao, largura, altura, corrigida):
    """
    K da imagem em que as deteções foram feitas: a "nova K" da correção da distorção (imagem
    corrigida) ou a K original (imagem sem correção; aproximado, porque ignora a distorção).
    calibracao = (K, D, nova_K, (largura, altura)) de leitura_bag. Ajusta à resolução da imagem.
    """
    if calibracao is None:
        return None
    k, _, nova_k, (wc, hc) = calibracao
    m = np.array(nova_k if corrigida else k, dtype=np.float64).copy()
    if (largura, altura) != (wc, hc):
        m = np.diag([largura / wc, altura / hc, 1.0]) @ m
    return m


class GeometriaCamara:
    """Uma câmara montada no carro: posição, orientação e intrínsecos (K) da imagem."""

    def __init__(self, cfg_cam, k):
        self.cfg = cfg_cam
        self.t = np.array([float(cfg_cam["x"]), float(cfg_cam["y"]), float(cfg_cam["z"])])
        self.rpy = orientacao(cfg_cam)
        self.r_corpo = rpy_para_matriz(*self.rpy)          # eixos do corpo da câmara no base_link
        self.r_otico = self.r_corpo @ OTICO_EM_CORPO       # eixos óticos no base_link
        self.k = np.asarray(k, dtype=np.float64)
        self.k_inv = np.linalg.inv(self.k)

    def raios(self, u, v):
        """Direção (no base_link) do raio de cada píxel."""
        pix = np.stack([np.asarray(u, float), np.asarray(v, float), np.ones(np.shape(u))], axis=-1)
        return (pix @ self.k_inv.T) @ self.r_otico.T

    def pixel_para_chao(self, u, v):
        """(x, y, válido): ponto do chão onde toca o raio de cada píxel (válido só abaixo do horizonte)."""
        d = self.raios(u, v)
        valido = d[..., 2] < -1e-6
        s = np.where(valido, -self.t[2] / np.where(valido, d[..., 2], -1.0), np.nan)
        return self.t[0] + s * d[..., 0], self.t[1] + s * d[..., 1], valido

    def chao_para_pixel(self, x, y):
        """(u, v, à frente): píxel onde aparece o ponto do chão (x, y); à frente = na frente da câmara."""
        p = np.stack([np.asarray(x, float) - self.t[0], np.asarray(y, float) - self.t[1],
                      np.full(np.shape(x), -self.t[2])], axis=-1)
        c = p @ self.r_otico                                # coordenadas no referencial ótico
        frente = c[..., 2] > 1e-3
        z = np.where(frente, c[..., 2], 1.0)
        uvw = c @ self.k.T
        return uvw[..., 0] / z, uvw[..., 1] / z, frente

    def distancia(self, x, y):
        """Distância horizontal (m) da câmara ao ponto do chão."""
        return np.hypot(np.asarray(x) - self.t[0], np.asarray(y) - self.t[1])

    def horizonte_v(self, u=None):
        """Linha (v) do horizonte na imagem, na coluna u (por omissão, o centro ótico)."""
        u = self.k[0, 2] if u is None else u
        # o horizonte é onde o raio fica horizontal (componente z = 0 no base_link)
        a = self.r_otico[2] @ self.k_inv                    # z(u, v) = a0*u + a1*v + a2
        return -(a[0] * u + a[2]) / a[1] if abs(a[1]) > 1e-9 else float("nan")

    # ---------------------------------------------------------------- TF
    def tf_corpo(self):
        """(translação, quaternion) de camara<N>_link no base_link."""
        return tuple(float(v) for v in self.t), matriz_para_quaternion(self.r_corpo)

    @staticmethod
    def tf_otico():
        """(translação, quaternion) de camara<N> (ótico) no camara<N>_link."""
        return (0.0, 0.0, 0.0), matriz_para_quaternion(OTICO_EM_CORPO)
