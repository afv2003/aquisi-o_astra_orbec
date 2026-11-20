# -*- coding: utf-8 -*-
"""
reproducao.py - Junta as imagens do bag original com as deteções de um registo.

Usado pelo reproduzir_registo.py (vídeo no ecrã ou em .mp4) e pelo gui_resultados.py (vídeo na
GUI), para os dois mostrarem exatamente o mesmo: as imagens lidas e corrigidas como no
processar_bag.py (leitura_bag.py) e as deteções registadas desenhadas com o visual.py.
Os modelos não voltam a correr.
"""
from pathlib import Path

import numpy as np

import leitura_bag
import registo
import visual

MODELOS = ("objetos", "linhas", "estrada")


class ErroRegisto(Exception):
    """Registo ou bag original em falta ou inválido (a mensagem explica o que fazer)."""


class Reproducao:
    def __init__(self, pasta_ensaio, bag=None):
        self.pasta = registo.pasta_do_ensaio(pasta_ensaio)
        self.ensaio = registo.ler_ensaio(self.pasta)
        if self.ensaio is None:
            raise ErroRegisto(f"'{self.pasta}' não tem ensaio.json (não é uma pasta de registo do processar_bag.py).")
        self.trechos = self.ensaio.get("trechos", [])
        if not self.trechos:
            raise ErroRegisto("O registo não tem trechos.")
        self.deteccoes = registo.ler_deteccoes(self.pasta / self.ensaio.get("ficheiro_deteccoes", "deteccoes.mcap"))

        # Bag original: onde estava no processamento, ou em data/<viagem>/ (ou o indicado)
        info_bag = self.ensaio["bag"]
        candidatos = [Path(bag)] if bag else [
            Path(info_bag["ficheiro"]),
            Path("data") / Path(info_bag["ficheiro"]).parent.name / info_bag["nome"],
        ]
        self.bag = next((c for c in candidatos if c.exists()), None)
        if self.bag is None:
            raise ErroRegisto("Não encontrei o bag original. Procurei em:\n  "
                              + "\n  ".join(str(c) for c in candidatos)
                              + "\nIndique onde está (no reproduzir_registo.py, com --bag <caminho do .mcap>).")
        erro = leitura_bag.verificar_mcap(self.bag)
        if erro:
            raise ErroRegisto(erro)
        tamanho = info_bag.get("tamanho_bytes")
        self.aviso_tamanho = (tamanho is not None and self.bag.stat().st_size != tamanho)
        self.indice_ok = leitura_bag.bag_tem_indice(self.bag)
        self.topico = info_bag["topico_imagem"]
        self.inicio_bag_ns = info_bag.get("inicio_ns") or min(t["inicio_ns"] for t in self.trechos)
        self.duracao_bag = info_bag.get("duracao_s") or 0.0

        self.corretor = None
        if self.ensaio.get("calibracao"):
            self.corretor = leitura_bag.CorretorDistorcao(leitura_bag.calibracao_de_dict(self.ensaio["calibracao"]))
        modelos = self.ensaio.get("modelos", {})
        self.nome_modelo = {k: modelos.get(k, {}).get("nome", k) for k in MODELOS}
        self.carregado = {k: bool(modelos.get(k, {}).get("carregado", True)) for k in MODELOS}
        self.camara = self.ensaio.get("camara")

    def trecho(self, n):
        return next(t for t in self.trechos if t["n"] == n)

    def imagens(self, trecho, desenhar=True):
        """
        Imagens de um trecho, já com as deteções desenhadas e o painel, como no processar_bag.py
        (ou, com desenhar=False, só corrigidas da distorção, sem desenhos).
        Gera (instante_ns, imagem BGR, dados das deteções ou None).
        """
        for msg in leitura_bag.ler_mensagens(self.bag, [self.topico], self.indice_ok,
                                             trecho["inicio_ns"], trecho["fim_ns"]):
            frame = leitura_bag.imagem_bgr(msg.ros_msg)
            if frame is None:
                continue
            dados = self.deteccoes.get(msg.log_time_ns)
            if not desenhar:
                yield msg.log_time_ns, self.corrigir(frame, dados), dados
                continue
            yield msg.log_time_ns, self.desenhar(frame, dados, trecho, msg.log_time_ns), dados

    def corrigir(self, frame, dados):
        """Correção da distorção, como estava no processamento dessa imagem."""
        por_omissao = self.corretor is not None
        corrigida = bool(dados.get("corrigida", por_omissao)) if dados else por_omissao
        return self.corretor(frame) if corrigida and self.corretor is not None else frame

    def desenhar(self, frame, dados, trecho, t_ns):
        por_omissao = self.corretor is not None
        corrigida = bool(dados.get("corrigida", por_omissao)) if dados else por_omissao
        if corrigida and self.corretor is not None:
            frame = self.corretor(frame)
        h, w = frame.shape[:2]

        caixas, linhas_pts, mascara = [], [], None
        ativos = dict(self.carregado)
        if dados:
            ativos = {k: bool(v) for k, v in (dados.get("modelos_ativos") or self.carregado).items()}
            caixas = [(*o["caixa"], o["classe"], o["confianca"]) for o in dados.get("objetos", [])]
            linhas_pts = [np.array(l, dtype=np.float32) for l in dados.get("linhas", []) if len(l) > 1]
            if ativos.get("estrada") and dados.get("estrada_contornos") is not None:
                mascara = registo.mascara_dos_contornos(dados["estrada_contornos"], dados.get("estrada_buracos", []), w, h)
        visual.desenhar_estrada(frame, mascara)
        visual.desenhar_linhas(frame, linhas_pts)
        visual.desenhar_caixas(frame, caixas)

        n_obj, n_lin = len(caixas), len(linhas_pts)
        pct = dados.get("estrada_percentagem", 0.0) if dados else 0.0
        valores = {"objetos": f"{n_obj} objeto" + ("" if n_obj == 1 else "s"),
                   "linhas": f"{n_lin} linha" + ("" if n_lin == 1 else "s"),
                   "estrada": f"{pct:.0%} da imagem"}
        info = {"tempo": (t_ns - self.inicio_bag_ns) / 1e9, "total": self.duracao_bag,
                "subtitulo": (f"Registo  ·  câmara {self.camara}  ·  trecho {trecho['n']}"
                              + ("" if dados else "  ·  sem deteções")),
                "cor_subtitulo": "#9CA3AF" if dados else "#F59E0B",
                "modelos": [(k, self.nome_modelo[k], valores[k], None, ativos.get(k, False), self.carregado[k])
                            for k in MODELOS],
                "rodape": f"{self.pasta.name}  ·  " + ("corrigida" if corrigida and self.corretor is not None
                                                        else "original")}
        return visual.compor(frame, caixas, info)
