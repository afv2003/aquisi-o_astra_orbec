# -*- coding: utf-8 -*-
"""
decisao.py - Decisão local: grelha local e os três avisos, a partir das deteções de cada imagem.

O mesmo código no online (processar_tempo_real.py, que publica os resultados em tópicos ROS) e no
offline (processar_bag.py, que os mostra na vista de cima), para os dois darem o mesmo:
  grelha local          grelha_local.py   (nav_msgs/OccupancyGrid para o planeador A* comum)
  faixas                aviso_faixa.py    (EstimadorFaixas: pelas linhas detetadas ou pelo centro)
  seguimento + TTC      seguimento.py     (objetos do YOLO seguidos entre imagens; tempo até contacto)
  mudança de faixa      aviso_faixa.py
  saída de faixa        aviso_saida_faixa.py
  colisão               aviso_colisao.py
Os parâmetros estão no config/montagem_camaras.yaml (secções grelha, aviso_faixa, aviso_saida_faixa,
aviso_colisao e seguimento).
"""
from . import aviso_colisao
from . import aviso_faixa
from . import aviso_saida_faixa
from . import grelha_local
from . import seguimento


class DecisaoLocal:
    def __init__(self, montagem):
        self.montagem = montagem
        self.grelha = grelha_local.GrelhaLocal(montagem)
        self.p_faixa = aviso_faixa.parametros(montagem)
        self.p_saida = aviso_saida_faixa.parametros(montagem)
        self.p_colisao = aviso_colisao.parametros(montagem)
        self.p_seguimento = dict(montagem.get("seguimento") or {})
        self.faixas = aviso_faixa.EstimadorFaixas(self.p_faixa)
        self.saida = aviso_saida_faixa.AvisoSaida(self.p_saida, self.grelha.carro)
        self.seguidores = {}
        self.linhas = {}            # câmara -> (linhas no chão, stamp_ns)
        self.objetos = {}           # câmara -> (objetos seguidos, stamp_ns)
        self.geos = {}              # câmara -> GeometriaCamara (posição na vista de cima)
        self.alturas = {}           # câmara -> altura da imagem (px)

    def _recentes(self, d, t_ns):
        v = self.grelha.validade_ns
        return {c: x for c, (x, t) in d.items() if abs(t_ns - t) <= v}

    def atualizar(self, cam, k, largura_img, altura_img, t_ns, mascara_estrada, caixas, linhas_img):
        """Uma imagem processada da câmara cam (k: intrínsecos da imagem das deteções). Devolve
        {grelha, faixas, faixa, saida, colisao, linhas_chao, geos}."""
        cam = str(cam)
        g = self.grelha
        g.atualizar(cam, k, largura_img, altura_img, t_ns, mascara_estrada, caixas)
        atual = g.combinada(t_ns)
        geo = g.geometria(cam, k)[0]
        self.geos[cam] = geo
        self.alturas[cam] = altura_img
        self.linhas[cam] = (g.linhas_no_chao(cam, k, linhas_img), t_ns)
        seg = self.seguidores.setdefault(cam, seguimento.Seguidor(self.p_seguimento))
        objs = seg.atualizar(caixas, t_ns, largura_img, altura_img)
        for o in objs:
            pos = g.posicao_objeto(geo, o["caixa"], o["classe"], largura_img)   # aproximada (chão plano)
            o["x"], o["y"] = (round(float(pos[0]), 2), round(float(pos[1]), 2)) if pos else (None, None)
            o["camara"], o["stamp_ns"] = cam, int(t_ns)
        self.objetos[cam] = (objs, t_ns)

        linhas = self._recentes(self.linhas, t_ns)
        faixas = self.faixas.atualizar(sum(linhas.values(), []), t_ns)
        objetos = sum(self._recentes(self.objetos, t_ns).values(), [])
        faixa = aviso_faixa.avaliar(g, atual, g.veiculos(t_ns), self.p_faixa, faixas, objetos)
        saida = self.saida.avaliar(self.faixas, t_ns, faixa)
        colisao = aviso_colisao.avaliar(objetos, faixas, self.p_colisao, self.p_faixa, g.carro, self.alturas)
        return {"grelha": atual, "faixas": faixas, "faixa": faixa, "saida": saida, "colisao": colisao,
                "linhas_chao": linhas, "geos": {c: v for c, v in self.geos.items() if c in linhas}}

    def remover(self, cam):
        """Câmara desligada: a parte dela fica desconhecida e os seus objetos esquecidos."""
        cam = str(cam)
        self.grelha.remover(cam)
        for d in (self.linhas, self.objetos, self.seguidores, self.geos):
            d.pop(cam, None)
