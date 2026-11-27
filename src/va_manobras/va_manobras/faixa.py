"""Ocupação da faixa: o primeiro obstáculo à frente, dentro da minha faixa.

Biblioteca sem ROS. Recebe os pontos ocupados de uma grelha local (base_link) e a linha de centro
da faixa no mesmo referencial, e devolve a distância livre ao longo da faixa.

    modo 'frenet'  um ponto está na faixa se |d| < meia_largura, com (s, d) medidos ao longo da estrada
    modo 'reto'    o mesmo, mas a fingir que a estrada é reta (s = x, d = y): é o que se faz sem
                   conhecer a estrada, e falha nas curvas
"""
from dataclasses import dataclass

import numpy as np

from .estrada import Estrada


@dataclass
class Ocupacao:
    distancia: float          # distância livre desde a frente do carro até ao obstáculo (m)
    livre: bool               # True se não há nada na faixa até ao alcance
    ponto: tuple = None       # (x, y) no base_link do ponto que limita a distância
    s_carro: float = 0.0      # posição do carro ao longo da linha de centro


def pontos_ocupados(grelha, limiar=50):
    """Centros (x, y) das células ocupadas de uma LocalGrid (ou objeto com os mesmos campos)."""
    iy, ix = np.nonzero(grelha.data >= limiar)
    return grelha.para_metros(ix, iy)


def ocupacao_da_faixa(x, y, estrada, meia_largura=1.75, x_frente=3.0, alcance=40.0, modo='frenet'):
    """Distância livre na faixa, a partir dos pontos ocupados (x, y) no base_link.

    estrada: Estrada no base_link (o carro está em (0, 0)); ignorada no modo 'reto'.
    x_frente: distância do base_link (eixo traseiro) à frente do carro.
    alcance: até onde se procura, medido a partir do base_link.
    """
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    if modo == 'reto':
        s, d, s_carro = x, y, 0.0
    else:
        s_carro = float(estrada.para_frenet(0.0, 0.0)[0][0])
        if x.size:
            s, d = estrada.para_frenet(x, y)
            s = s - s_carro
        else:
            s = d = x
    na_faixa = (np.abs(d) < meia_largura) & (s > x_frente) & (s < alcance)
    if not np.any(na_faixa):
        return Ocupacao(alcance - x_frente, True, None, s_carro)
    k = np.flatnonzero(na_faixa)[np.argmin(s[na_faixa])]
    return Ocupacao(float(s[k] - x_frente), False, (float(x[k]), float(y[k])), s_carro)


def corredor(estrada, meia_largura, s_carro, s_ini, s_fim, passo=1.0, modo='frenet'):
    """Contorno da faixa entre s_carro + s_ini e s_carro + s_fim, para desenhar (lista de (x, y))."""
    if modo == 'reto':
        return [(s_ini, -meia_largura), (s_fim, -meia_largura),
                (s_fim, meia_largura), (s_ini, meia_largura)]
    s = np.arange(s_carro + s_ini, s_carro + s_fim + 1e-9, passo)
    xe, ye = estrada.de_frenet(s, meia_largura)
    xd, yd = estrada.de_frenet(s[::-1], -meia_largura)
    return list(zip(np.r_[xe, xd].tolist(), np.r_[ye, yd].tolist()))


__all__ = ['Estrada', 'Ocupacao', 'pontos_ocupados', 'ocupacao_da_faixa', 'corredor']
