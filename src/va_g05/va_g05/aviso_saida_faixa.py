# -*- coding: utf-8 -*-
"""
aviso_saida_faixa.py - Aviso de saída de faixa (Lane Departure Warning) com as câmaras laterais (G5).

As câmaras laterais veem, de cada lado, a linha da própria faixa junto ao carro (é o que um condutor
vê no retrovisor quando se encosta à linha). As linhas detetadas (CLRNet) são projetadas no chão
(geometria aproximada, chão plano: grelha_local.linhas_no_chao) e ajustadas a uma reta
y = a + b x no base_link (aviso_faixa.EstimadorFaixas, o mesmo do aviso de mudança de faixa). Daí:

  distância à linha   o espaço entre o lado do carro (carro.largura / 2) e a linha, no ponto do carro
                      mais perto dela (da traseira à frente: com a linha inclinada em relação ao carro,
                      a frente ou a traseira chega primeiro). Aproximada (geometria do chão);
  tempo até à linha   (TLC, time to line crossing) se a distância está a diminuir: distância / ritmo a
                      que diminui, numa reta de mínimos quadrados das últimas amostras (janela_s).

Estado de cada lado (o mais grave primeiro):
  A_PISAR       a linha está debaixo do carro (distância <= 0): o carro está a sair da faixa;
  PERTO         a menos de limiar_perto_m da linha;
  A_APROXIMAR   a aproximar-se da linha, a menos de tlc_aviso_s de lhe chegar;
  OK            longe da linha e sem se aproximar;
  SEM_LINHA     a linha desse lado não é vista (sem linhas pintadas, ou a deteção falhou).
Com a faixa desse lado OCUPADA (aviso de mudança de faixa), uma saída para esse lado é PERIGO.

Não se sabe se a saída é intencional (pisca): sem o estado do pisca (CAN do carro, ou outro grupo), o
aviso dispara também numa mudança de faixa voluntária.
"""
from collections import deque

import numpy as np

LADOS = ("esquerda", "direita")
SINAL = {"esquerda": 1.0, "direita": -1.0}
POR_OMISSAO = {"limiar_perto_m": 0.3, "tlc_aviso_s": 1.0, "janela_s": 1.0, "min_amostras": 3,
               "min_ritmo_m_s": 0.05, "distancia_max_m": 2.5}
ORDEM = {"A_PISAR": 4, "PERTO": 3, "A_APROXIMAR": 2, "OK": 1, "SEM_LINHA": 0}


def parametros(montagem):
    p = dict(POR_OMISSAO)
    p.update((montagem or {}).get("aviso_saida_faixa") or {})
    return p


class AvisoSaida:
    def __init__(self, p, carro):
        """carro: (trás, frente, largura) em m (GrelhaLocal.carro)."""
        self.p = p
        self.tras, self.frente, self.largura = carro
        self.hist = {lado: deque() for lado in LADOS}

    def avaliar(self, estimador, t_ns, mudanca=None):
        """estimador: aviso_faixa.EstimadorFaixas (as linhas da própria faixa, de cada lado, sempre
        estimadas, qualquer que seja o modo_faixa); mudanca: o aviso de mudança de faixa (para o PERIGO)."""
        t_s = t_ns / 1e9
        res = {}
        for lado in LADOS:
            s = SINAL[lado]
            linha = estimador._recente((lado, "interior"), t_ns)
            info = {"estado": "SEM_LINHA"}
            h = self.hist[lado]
            if linha is None:
                h.clear()
                res[lado] = info
                continue
            a, b = linha
            # distância do lado do carro à linha, no ponto do carro mais perto dela
            dist = min(s * (a + b * x) for x in (self.tras, self.frente)) - self.largura / 2
            if dist > float(self.p["distancia_max_m"]):
                h.clear()
                res[lado] = info                    # a linha "interior" está longe: não é a da faixa
                continue
            h.append((t_s, dist))
            while h and t_s - h[0][0] > float(self.p["janela_s"]):
                h.popleft()
            tlc = None
            if len(h) >= int(self.p["min_amostras"]) and h[-1][0] - h[0][0] > 0.2:
                t = np.array([v[0] for v in h])
                ritmo = np.polyfit(t - t[0], np.array([v[1] for v in h]), 1)[0]     # m/s (< 0: a aproximar)
                if ritmo < -float(self.p["min_ritmo_m_s"]) and dist > 0:
                    tlc = dist / -ritmo
            info.update(distancia_m=round(float(dist), 2), tlc_s=round(float(tlc), 2) if tlc is not None else None,
                        linha=[round(a, 3), round(b, 4)])
            if dist <= 0:
                info["estado"] = "A_PISAR"
            elif dist < float(self.p["limiar_perto_m"]):
                info["estado"] = "PERTO"
            elif tlc is not None and tlc < float(self.p["tlc_aviso_s"]):
                info["estado"] = "A_APROXIMAR"
            else:
                info["estado"] = "OK"
            ocupada = (mudanca or {}).get(lado, {}).get("estado") == "OCUPADA"
            info["perigo"] = bool(ocupada and info["estado"] in ("A_PISAR", "PERTO", "A_APROXIMAR"))
            res[lado] = info
        return res

    def limpar(self):
        for h in self.hist.values():
            h.clear()


def texto(info, ascii=False):
    e = info.get("estado", "SEM_LINHA")
    if e == "SEM_LINHA":
        return "SEM LINHA"
    d = info.get("distancia_m")
    if e == "A_PISAR":
        t = "A PISAR A LINHA"
    elif e == "PERTO":
        t = f"PERTO ({d:.1f} m)"
    elif e == "A_APROXIMAR":
        t = f"A APROXIMAR ({info['tlc_s']:.1f} s, {d:.1f} m)"
    else:
        t = f"OK ({d:.1f} m)"
    if info.get("perigo"):
        t += " + FAIXA OCUPADA"
    return t


COR = {"OK": (80, 190, 80), "A_APROXIMAR": (0, 200, 255), "PERTO": (0, 140, 255), "A_PISAR": (40, 40, 220),
       "SEM_LINHA": (140, 140, 140)}
