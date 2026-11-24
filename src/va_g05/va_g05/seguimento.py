# -*- coding: utf-8 -*-
"""
seguimento.py - Seguimento dos objetos do YOLO entre imagens da mesma câmara e tempo até contacto.

Para o aviso de colisão e para os veículos que se aproximam na faixa ao lado (aviso_faixa.py) é
preciso saber se um objeto se está a APROXIMAR, e quão depressa, sem medir distâncias nem velocidades
com a câmara. Usa-se o que a imagem dá diretamente: o TAMANHO do objeto na imagem.

* Seguimento: cada caixa nova é associada à caixa da imagem anterior que mais se lhe sobrepõe (IoU),
  com a mesma categoria (veículo, peão, bicicleta); as que não se associam são objetos novos. Um objeto
  que deixa de ser visto durante mais de max_ausencia_s é esquecido.
* Tempo até contacto (TTC): se um objeto se aproxima da câmara, o seu tamanho na imagem cresce.
  Com s = altura da caixa, o tempo até chegar à câmara (ao plano da câmara) é
      TTC = s / (ds/dt) = 1 / (d ln(s) / dt)
  É a "expansão" da imagem (looming): não precisa da distância nem da velocidade, só do ritmo a que a
  caixa cresce. Usa-se a altura (e não a largura) porque, visto de lado, a largura da caixa inclui o
  comprimento do veículo, que não cresce ao mesmo ritmo. Calcula-se com uma reta de mínimos quadrados
  de ln(s) ao longo do tempo, nas últimas amostras (janela_s), para não reagir ao tremor de uma só
  caixa; a reta dá o TTC no meio da janela, que se passa para o instante atual (menos o tempo entre os
  dois). TTC só existe (positivo) se o objeto cresce; um objeto que se afasta ou mantém a distância tem
  TTC None.
  Com o carro em andamento, um objeto parado atrás afasta-se (encolhe): não dá aviso, como deve ser.
* Uma caixa cortada pela borda da imagem não serve para o TTC (o tamanho não é o real).

Limites: precisa de várias imagens do mesmo objeto (com o CPU, a ~1 imagem por segundo por câmara,
o TTC demora alguns segundos a aparecer; com GPU, uma fração de segundo); o tremor das caixas do YOLO
dá erros de ~10-20 % no TTC.
"""
from collections import deque

import numpy as np

VEICULOS = {"car", "truck", "bus", "motorcycle"}
CATEGORIA = {**{c: "veiculo" for c in VEICULOS}, "person": "peao", "bicycle": "bicicleta"}
POR_OMISSAO = {"iou_minimo": 0.25, "max_ausencia_s": 0.8, "janela_s": 1.5, "min_amostras": 3,
               "min_intervalo_s": 0.25, "ttc_maximo_s": 8.0, "margem_borda_px": 3}


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    uniao = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / uniao if uniao > 0 else 0.0


class Objeto:
    def __init__(self, ident, caixa, classe, conf, t_s, cortada):
        self.id = ident
        self.classe, self.conf = classe, conf
        self.caixa = caixa
        self.visto_s = t_s
        self.n = 0
        self.hist = deque()             # (t_s, ln(s)) só de caixas não cortadas
        self.juntar(caixa, classe, conf, t_s, cortada)

    def juntar(self, caixa, classe, conf, t_s, cortada):
        self.caixa, self.classe, self.conf, self.visto_s = caixa, classe, conf, t_s
        self.n += 1
        if not cortada:
            self.hist.append((t_s, float(np.log(max(caixa[3] - caixa[1], 1.0)))))

    def ttc(self, p, agora_s):
        while self.hist and agora_s - self.hist[0][0] > p["janela_s"]:
            self.hist.popleft()
        if len(self.hist) < int(p["min_amostras"]):
            return None
        t = np.array([h[0] for h in self.hist])
        ls = np.array([h[1] for h in self.hist])
        if t[-1] - t[0] < p["min_intervalo_s"]:
            return None
        declive = np.polyfit(t - t[0], ls, 1)[0]          # d ln(s) / dt, no meio da janela
        if declive <= 1.0 / p["ttc_maximo_s"]:
            return None                                     # não se aproxima (ou muito devagar)
        return float(max(1.0 / declive - (t[-1] - t.mean()), 0.0))


class Seguidor:
    """Seguimento dos objetos de uma câmara."""

    def __init__(self, parametros=None):
        self.p = dict(POR_OMISSAO)
        self.p.update(parametros or {})
        self.objetos = []
        self._proximo = 1

    def atualizar(self, caixas, t_ns, largura_img, altura_img):
        """caixas: [(x1, y1, x2, y2, classe, confiança)] do YOLO; devolve a lista dos objetos vistos nesta
        imagem: {id, classe, categoria, caixa, conf, ttc_s, n}."""
        t_s = t_ns / 1e9
        m = self.p["margem_borda_px"]
        novas = [c for c in caixas if c[4] in CATEGORIA]
        pares = sorted(((iou(o.caixa, c[:4]), i, j) for i, o in enumerate(self.objetos)
                        for j, c in enumerate(novas) if CATEGORIA[o.classe] == CATEGORIA[c[4]]), reverse=True)
        usados_o, usados_c, vistos = set(), set(), []
        for v, i, j in pares:
            if v < self.p["iou_minimo"] or i in usados_o or j in usados_c:
                continue
            usados_o.add(i)
            usados_c.add(j)
            c = novas[j]
            cortada = c[0] <= m or c[1] <= m or c[2] >= largura_img - 1 - m or c[3] >= altura_img - 1 - m
            self.objetos[i].juntar(tuple(map(float, c[:4])), c[4], float(c[5]), t_s, cortada)
            vistos.append(self.objetos[i])
        for j, c in enumerate(novas):
            if j in usados_c:
                continue
            cortada = c[0] <= m or c[1] <= m or c[2] >= largura_img - 1 - m or c[3] >= altura_img - 1 - m
            o = Objeto(self._proximo, tuple(map(float, c[:4])), c[4], float(c[5]), t_s, cortada)
            self._proximo += 1
            self.objetos.append(o)
            vistos.append(o)
        self.objetos = [o for o in self.objetos if t_s - o.visto_s <= self.p["max_ausencia_s"]]
        res = []
        for o in vistos:
            ttc = o.ttc(self.p, t_s)
            res.append({"id": o.id, "classe": o.classe, "categoria": CATEGORIA[o.classe],
                        "caixa": [round(v, 1) for v in o.caixa], "conf": round(o.conf, 2),
                        "ttc_s": round(ttc, 2) if ttc is not None else None, "n": o.n})
        return res

    def limpar(self):
        self.objetos = []
