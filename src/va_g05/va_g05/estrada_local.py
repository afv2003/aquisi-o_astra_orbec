# -*- coding: utf-8 -*-
"""
estrada_local.py - A estrada (/estrada/centro, contrato da aula 11) no referencial do carro, em Frenet.

O contrato comum da estrada (aula 11): nav_msgs/Path no base_link com a linha de centro da faixa do carro,
de trás para a frente (header.stamp = instante da pose ou da deteção que lhe deu origem). Quem a produz
pode ser outro grupo (rota gravada com GNSS ou odometria, linhas da câmara da frente, bermas do LiDAR).

Com ela, "em que faixa está" deixa de supor a estrada reta: cada ponto (x, y) no base_link passa a
(s, d) de Frenet (biblioteca comum va_manobras.estrada, sem alterações): s ao longo da estrada (aqui
relativo ao carro: s = 0 à altura do base_link, negativo para trás) e d para o lado (positivo à esquerda).
A faixa do carro é |d| < largura_faixa / 2 e a faixa ao lado vai de largura/2 a 1,5 x largura (com sinal).
É o que os avisos usam (aviso_faixa.py, aviso_saida_faixa.py, aviso_colisao.py) quando há estrada recente.
"""
import sys
from pathlib import Path

import numpy as np

try:
    from va_manobras.estrada import Estrada          # no Docker: instalado pelo colcon
except ImportError:                                   # fora do Docker (offline): a partir do código-fonte
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "va_manobras"))
    from va_manobras.estrada import Estrada  # noqa: E402


class EstradaLocal:
    """Linha de centro da faixa no base_link (pontos de /estrada/centro), num instante."""

    def __init__(self, xs, ys, stamp_ns):
        self.estrada = Estrada(xs, ys, fechada=False)
        self.stamp_ns = int(stamp_ns)
        self.s_carro = float(self.estrada.para_frenet(0.0, 0.0)[0][0])
        self.s_min = -self.s_carro                                # até onde vai para trás (s relativo)
        self.s_max = self.estrada.comprimento - self.s_carro      # e para a frente
        self._cache = {}

    def frenet(self, x, y):
        """(s, d) dos pontos (x, y) no base_link, com s relativo ao carro. Antes do primeiro ponto e depois
        do último, a estrada continua em linha reta (s < 0 ou s > comprimento, em vez de ficar parado)."""
        forma = np.shape(x)
        x, y = np.asarray(x, float).ravel(), np.asarray(y, float).ravel()
        e = self.estrada
        s, d = e.para_frenet(x, y)
        u0 = (x - e.p[0, 0]) * e.t[0, 0] + (y - e.p[0, 1]) * e.t[0, 1]
        antes = (s <= 1e-9) & (u0 < 0)
        s[antes] = u0[antes]
        un = (x - e.p[-1, 0]) * e.t[-1, 0] + (y - e.p[-1, 1]) * e.t[-1, 1]
        depois = (s >= e.comprimento - 1e-9) & (un > 0)
        s[depois] = e.comprimento + un[depois]
        return (s - self.s_carro).reshape(forma), d.reshape(forma)

    def de_frenet(self, s, d):
        """Pontos (x, y) no base_link a s (relativo ao carro) ao longo da estrada e d para o lado (também
        antes do primeiro ponto e depois do último, em linha reta)."""
        e = self.estrada
        s = np.atleast_1d(np.asarray(s, float)) + self.s_carro
        d = np.broadcast_to(np.asarray(d, float), s.shape)
        x, y = (np.asarray(v, float) for v in e.de_frenet(s, d))
        for sel, p, t, s0 in ((s < 0, e.p[0], e.t[0], 0.0), (s > e.comprimento, e.p[-1], e.t[-1], e.comprimento)):
            if sel.any():
                u = s[sel] - s0
                x[sel] = p[0] + u * t[0] - d[sel] * t[1]
                y[sel] = p[1] + u * t[1] + d[sel] * t[0]
        return x, y

    def frenet_celulas(self, grelha, x_min, x_max):
        """(s, d) dos centros das células da grelha (GrelhaLocal), calculado uma vez por estrada; só para as
        células com x entre x_min e x_max (as outras ficam NaN), que é onde estão as zonas dos avisos."""
        chave = (id(grelha), x_min, x_max)
        if chave not in self._cache:
            s = np.full(grelha.cx.shape, np.nan)
            d = np.full(grelha.cx.shape, np.nan)
            sel = (grelha.cx >= x_min) & (grelha.cx <= x_max)
            if sel.any():
                s[sel], d[sel] = self.frenet(grelha.cx[sel], grelha.cy[sel])
            self._cache[chave] = (s, d)
        return self._cache[chave]

    def borda(self, d, s_ini, s_fim, passo=1.0):
        """Linha paralela ao centro, a d para o lado, de s_ini a s_fim (relativos): [(x, y), ...]."""
        s = np.arange(s_ini, s_fim + 1e-9, passo)
        if s.size < 2:
            s = np.array([s_ini, s_fim])
        x, y = self.de_frenet(s, d)
        return list(zip(np.asarray(x).tolist(), np.asarray(y).tolist()))

    def reta_local(self, d, x_ini, x_fim):
        """A borda a d para o lado, perto do carro (s de x_ini a x_fim), ajustada a uma reta y = a + b x
        (para o aviso de saída de faixa, que trabalha com retas). Devolve (a, b)."""
        pts = np.array(self.borda(d, x_ini, x_fim, 0.5))
        b, a = np.polyfit(pts[:, 0], pts[:, 1], 1)
        return float(a), float(b)


def de_mensagem(xs, ys, stamp_ns):
    """EstradaLocal a partir dos pontos de um nav_msgs/Path; None se tiver menos de 2 pontos distintos."""
    try:
        return EstradaLocal(xs, ys, stamp_ns)
    except ValueError:
        return None
