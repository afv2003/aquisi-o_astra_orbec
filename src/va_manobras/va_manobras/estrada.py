"""A estrada como referência: linha de centro da faixa e coordenadas de Frenet (s, d).

Biblioteca sem ROS (só numpy), partilhada por todos os grupos.

A linha de centro é uma poligonal [(x0, y0), (x1, y1), ...] num referencial qualquer
(odom, map ou base_link). Para cada ponto (x, y):
    s  distância percorrida ao longo da linha de centro, desde o primeiro ponto
    d  distância lateral com sinal: positiva para a esquerda da linha, negativa para a direita
É o referencial de Frenet da aula teórica 11: "estar na minha faixa" passa a ser |d| < meia largura,
seja a estrada reta ou em curva.
"""
import csv

import numpy as np


class Estrada:

    def __init__(self, x, y, fechada=None):
        p = np.column_stack([np.asarray(x, float), np.asarray(y, float)])
        passo = np.hypot(*np.diff(p, axis=0).T) if len(p) > 1 else np.zeros(0)
        p = p[np.r_[True, passo > 1e-6]]                      # sem pontos repetidos
        if len(p) < 2:
            raise ValueError('a linha de centro precisa de pelo menos 2 pontos distintos')
        if fechada is None:                                    # volta completa: fim junto ao início
            fechada = len(p) > 3 and np.hypot(*(p[0] - p[-1])) < 2.0
        if fechada and np.hypot(*(p[0] - p[-1])) > 1e-6:
            p = np.vstack([p, p[:1]])
        self.p = p
        self.fechada = bool(fechada)
        seg = np.diff(p, axis=0)
        self.comp_seg = np.hypot(seg[:, 0], seg[:, 1])
        self.t = seg / self.comp_seg[:, None]                  # tangente unitária de cada segmento
        self.s = np.r_[0.0, np.cumsum(self.comp_seg)]          # s no início de cada segmento
        self.comprimento = float(self.s[-1])

    # ------------------------------------------------------------ (x, y) <-> (s, d)
    def para_frenet(self, x, y):
        """Projeta pontos (x, y) na linha de centro. Devolve arrays (s, d)."""
        x = np.atleast_1d(np.asarray(x, float))
        y = np.atleast_1d(np.asarray(y, float))
        a, t, L = self.p[:-1], self.t, self.comp_seg
        dx = x[:, None] - a[None, :, 0]                        # (pontos, segmentos)
        dy = y[:, None] - a[None, :, 1]
        u = np.clip(dx * t[:, 0] + dy * t[:, 1], 0.0, L)       # posição ao longo de cada segmento
        ex = dx - u * t[:, 0]
        ey = dy - u * t[:, 1]
        j = np.argmin(ex * ex + ey * ey, axis=1)               # segmento mais próximo
        k = np.arange(x.size)
        s = self.s[j] + u[k, j]
        d = t[j, 0] * dy[k, j] - t[j, 1] * dx[k, j]            # produto externo: esquerda > 0
        return s, d

    def de_frenet(self, s, d=0.0):
        """Pontos (x, y) à distância s ao longo da linha e d para o lado."""
        s = np.atleast_1d(np.asarray(s, float))
        d = np.broadcast_to(np.asarray(d, float), s.shape)
        if self.fechada:
            s = np.mod(s, self.comprimento)
        else:
            s = np.clip(s, 0.0, self.comprimento)
        j = np.clip(np.searchsorted(self.s, s, side='right') - 1, 0, len(self.t) - 1)
        u = s - self.s[j]
        t = self.t[j]
        x = self.p[j, 0] + u * t[:, 0] - d * t[:, 1]
        y = self.p[j, 1] + u * t[:, 1] + d * t[:, 0]
        return x, y

    def rumo(self, s):
        """Orientação (rad) da linha de centro em s."""
        s = np.atleast_1d(np.asarray(s, float))
        if self.fechada:
            s = np.mod(s, self.comprimento)
        j = np.clip(np.searchsorted(self.s, s, side='right') - 1, 0, len(self.t) - 1)
        return np.arctan2(self.t[j, 1], self.t[j, 0])

    def trecho(self, s0, s1, passo=1.0):
        """Pontos da linha de centro entre s0 e s1 (numa volta fechada, s pode passar o fim)."""
        if not self.fechada:
            s0, s1 = max(s0, 0.0), min(s1, self.comprimento)
        s = np.arange(s0, s1 + 1e-9, passo)
        return self.de_frenet(s, 0.0)

    # ------------------------------------------------------------ ficheiros
    @classmethod
    def de_csv(cls, caminho):
        """Lê uma rota em CSV com colunas x,y (linha de cabeçalho opcional)."""
        xs, ys = [], []
        with open(caminho, newline='') as f:
            for linha in csv.reader(f):
                if not linha or linha[0].strip().startswith('#'):
                    continue
                try:
                    xs.append(float(linha[0]))
                    ys.append(float(linha[1]))
                except ValueError:                             # cabeçalho
                    continue
        return cls(xs, ys)

    def para_csv(self, caminho, passo=1.0):
        x, y = self.trecho(0.0, self.comprimento, passo)
        with open(caminho, 'w', newline='') as f:
            f.write('x,y\n')
            for xi, yi in zip(x, y):
                f.write(f'{xi:.3f},{yi:.3f}\n')


def pista_oval(reta=120.0, raio=35.0, passo=1.0):
    """Pista de teste fechada: duas retas e duas curvas de raio `raio`, sentido anti-horário.

    Começa em (0, 0) virada para +x, que é onde o simulador põe o carro.
    """
    pts = []
    for s in np.arange(0.0, reta, passo):
        pts.append((s, 0.0))
    for a in np.arange(0.0, np.pi, passo / raio):
        pts.append((reta + raio * np.sin(a), raio - raio * np.cos(a)))
    for s in np.arange(0.0, reta, passo):
        pts.append((reta - s, 2 * raio))
    for a in np.arange(0.0, np.pi, passo / raio):
        pts.append((-raio * np.sin(a), raio + raio * np.cos(a)))
    x, y = np.array(pts).T
    return Estrada(x, y, fechada=True)
