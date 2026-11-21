# -*- coding: utf-8 -*-
"""
ufld.py - Deteção de linhas da estrada com o UFLDv2 (Ultra-Fast-Lane-Detection-v2) em ONNX.

Usado pelo comparar_linhas.py, para comparar com o CLRNet (linhas.py). Mesma interface do
linhas.DetetorLinhas: detetar(imagem BGR) devolve uma lista de linhas, cada uma um array (N, 2)
de pontos (x, y) em píxeis da imagem recebida.

Modelo: UFLDv2 ResNet34 treinado no CULane (F1 76.0), o mesmo conjunto de treino e o mesmo backbone
do CLRNet usado no projeto. O ONNX é o convertido pelo PINTO0309 (indicado no README oficial do
UFLDv2): ufldv2_culane_res34_320x1600.onnx, guardado como modelos/ufld.onnx pelo preparar_ufld.py
(é demasiado grande para o git). A configuração está em modelos/ufld.json.

Pré e pós-processamento iguais aos do demo oficial (cfzd/Ultra-Fast-Lane-Detection-v2, demo.py e
configs/culane_res34.py): imagem RGB normalizada com a média e o desvio do ImageNet; a imagem é
reduzida para 1600 x 533 e ficam as 320 linhas de baixo (crop_ratio 0.6, ou seja, corta-se o topo,
40% da altura); a saída tem, para 4 linhas, a posição em 72 linhas de âncora (as 2 do meio, as da
faixa do veículo) e em 81 colunas de âncora (as 2 de fora); uma linha só conta se existir em mais
de metade das âncoras (linhas do meio) ou de um quarto (linhas de fora).
"""
import json
from pathlib import Path

import cv2
import numpy as np

import gpu

PASTA_MODELOS = Path(__file__).resolve().parent / "modelos"
MEDIA_IMAGENET = np.array([0.485, 0.456, 0.406], dtype=np.float32)
DESVIO_IMAGENET = np.array([0.229, 0.224, 0.225], dtype=np.float32)
SAIDAS = ("loc_row", "loc_col", "exist_row", "exist_col")

# Valores por omissão = configs/culane_res34.py do UFLDv2 (e utils/common.py, âncoras do CULane)
CONFIG_OMISSAO = {
    "img_w": 1600,              # largura da entrada do modelo
    "img_h": 320,               # altura da entrada do modelo
    "crop_ratio": 0.6,          # parte de baixo da imagem que o modelo vê (no treino)
    "num_row": 72,              # âncoras em linha (para as 2 linhas do meio)
    "num_col": 81,              # âncoras em coluna (para as 2 linhas de fora)
    "row_anchor": [0.42, 1.0],  # posição das âncoras em linha, em fração da altura da imagem
    "col_anchor": [0.0, 1.0],   # posição das âncoras em coluna, em fração da largura
    "linhas_row": [1, 2],       # linhas descodificadas pelas âncoras em linha (as do meio)
    "linhas_col": [0, 3],       # linhas descodificadas pelas âncoras em coluna (as de fora)
    "min_row": 0.5,             # uma linha do meio conta se existir em mais de 50% das âncoras
    "min_col": 0.25,            # uma linha de fora conta se existir em mais de 25% das âncoras
    "local_width": 1,           # vizinhos usados na média da posição (como no demo oficial)
    "corte_topo": None,         # None = 1 - crop_ratio (0.4), como no treino
}


def carregar_config(caminho_json=None):
    cfg = dict(CONFIG_OMISSAO)
    caminho_json = Path(caminho_json) if caminho_json else PASTA_MODELOS / "ufld.json"
    if caminho_json.exists():
        cfg.update(json.loads(caminho_json.read_text(encoding="utf-8")))
    if cfg.get("corte_topo") is None:
        cfg["corte_topo"] = 1.0 - cfg["crop_ratio"]
    return cfg


def preprocessar(img_bgr, cfg):
    """Imagem BGR (H, W, 3) -> tensor (1, 3, img_h, img_w), como no demo oficial; devolve também o corte."""
    h = img_bgr.shape[0]
    corte_px = int(round(h * cfg["corte_topo"]))
    recorte = img_bgr[corte_px:, :, ::-1]                              # corta o topo; BGR -> RGB
    redim = cv2.resize(recorte, (cfg["img_w"], cfg["img_h"]), interpolation=cv2.INTER_LINEAR)
    x = (redim.astype(np.float32) / 255.0 - MEDIA_IMAGENET) / DESVIO_IMAGENET
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None]), corte_px


def _softmax(v):
    e = np.exp(v - v.max())
    return e / e.sum()


def _posicao(loc, m, k, i, local_width):
    """Média da posição em torno do máximo (softmax dos vizinhos), como o pred2coords oficial."""
    n_grid = loc.shape[0]
    ind = np.arange(max(0, m - local_width), min(n_grid - 1, m + local_width) + 1)
    p = _softmax(loc[ind, k, i].astype(np.float64))
    return ((p * ind).sum() + 0.5) / (n_grid - 1)


def descodificar(saidas, cfg, largura, altura):
    """
    Saídas do modelo ({loc_row, loc_col, exist_row, exist_col}, com o lote) -> linhas, cada uma um
    array (N, 2) de pontos (x, y) em píxeis da imagem recebida (largura x altura), pela ordem da
    esquerda para a direita. As posições do modelo são frações da imagem "do treino" (a imagem toda,
    da qual o modelo vê os 60% de baixo); com outro corte do topo, são convertidas para a imagem real.
    """
    loc_row, loc_col = saidas["loc_row"][0], saidas["loc_col"][0]          # (200, 72, 4), (100, 81, 4)
    exist_row, exist_col = saidas["exist_row"][0], saidas["exist_col"][0]  # (2, 72, 4), (2, 81, 4)
    num_row, num_col = loc_row.shape[1], loc_col.shape[1]
    row_anchor = np.linspace(cfg["row_anchor"][0], cfg["row_anchor"][1], num_row)
    col_anchor = np.linspace(cfg["col_anchor"][0], cfg["col_anchor"][1], num_col)
    max_row, valid_row = loc_row.argmax(0), exist_row.argmax(0)            # (72, 4)
    max_col, valid_col = loc_col.argmax(0), exist_col.argmax(0)            # (81, 4)
    lw = int(cfg["local_width"])

    # fração da altura "do treino" -> píxel da imagem real (com o corte do topo usado)
    visto = 1.0 - cfg["crop_ratio"]
    corte = cfg["corte_topo"]

    def y_real(f):
        return (corte + (f - visto) / cfg["crop_ratio"] * (1.0 - corte)) * altura

    linhas = {}
    for i in cfg["linhas_row"]:
        if valid_row[:, i].sum() > num_row * cfg["min_row"]:
            pts = [(_posicao(loc_row, max_row[k, i], k, i, lw) * largura, y_real(row_anchor[k]))
                   for k in range(num_row) if valid_row[k, i]]
            linhas[i] = pts
    for i in cfg["linhas_col"]:
        if valid_col[:, i].sum() > num_col * cfg["min_col"]:
            pts = [(col_anchor[k] * largura, y_real(_posicao(loc_col, max_col[k, i], k, i, lw)))
                   for k in range(num_col) if valid_col[k, i]]
            linhas[i] = pts
    resultado = []
    for i in sorted(linhas):                                              # 0..3: da esquerda para a direita
        pts = np.array(linhas[i], dtype=np.float32)
        pts = pts[(pts[:, 0] >= 0) & (pts[:, 0] < largura) & (pts[:, 1] >= 0) & (pts[:, 1] < altura)]
        if len(pts) > 1:
            resultado.append(pts[np.argsort(-pts[:, 1])])                 # de baixo para cima, como o CLRNet
    return resultado


class DetetorUFLD:
    """Mesma interface do linhas.DetetorLinhas (dispositivo, cfg, detetar)."""

    def __init__(self, caminho_onnx=None, caminho_json=None, corte_topo=None):
        import onnxruntime as ort
        self.caminho = Path(caminho_onnx) if caminho_onnx else PASTA_MODELOS / "ufld.onnx"
        if not self.caminho.exists():
            raise FileNotFoundError(self.caminho)
        self.cfg = carregar_config(caminho_json or self.caminho.with_suffix(".json"))
        if corte_topo is not None:
            self.cfg["corte_topo"] = corte_topo
        self.ort = ort
        preferidos = gpu.preparar(ort)   # GPU se houver (CUDA ou DirectML), senão CPU (gpu.py)
        try:
            self._criar_sessao(preferidos)
        except Exception as e:
            print(f"Aviso: não foi possível usar a GPU no UFLD ({type(e).__name__}); a usar o CPU.")
            self._criar_sessao(["CPUExecutionProvider"])
        self.dispositivo = gpu.confirmar(self.sessao, "UFLD")   # a GPU pode só falhar ao correr

    def _criar_sessao(self, fornecedores):
        opcoes = self.ort.SessionOptions()
        opcoes.log_severity_level = 3
        self.sessao = self.ort.InferenceSession(str(self.caminho), sess_options=opcoes, providers=fornecedores)
        entrada = self.sessao.get_inputs()[0]
        self.entrada = entrada.name
        # tamanho da entrada lido do próprio modelo (1600 x 320 no CULane)
        if isinstance(entrada.shape[2], int) and isinstance(entrada.shape[3], int):
            self.cfg["img_h"], self.cfg["img_w"] = int(entrada.shape[2]), int(entrada.shape[3])
        nomes = [s.name for s in self.sessao.get_outputs()]
        # as saídas têm os nomes do pt2onnx.py oficial; se não tiverem, usa-se a ordem
        self.nomes_saida = nomes if set(SAIDAS) <= set(nomes) else list(SAIDAS)
        self._por_ordem = not set(SAIDAS) <= set(nomes)
        self.dispositivo = gpu.dispositivo(self.sessao)

    def _correr(self, x):
        saidas = self.sessao.run(None, {self.entrada: x})
        if self._por_ordem:
            return dict(zip(SAIDAS, saidas))
        return dict(zip(self.nomes_saida, saidas))

    def detetar(self, img_bgr):
        """Devolve uma lista de linhas; cada linha é um array (N, 2) de pontos (x, y) em píxeis."""
        x, _ = preprocessar(img_bgr, self.cfg)
        try:
            saidas = self._correr(x)
        except Exception as e:
            if self.dispositivo == "CPU":
                raise
            print(f"\nAviso: o UFLD falhou na {self.dispositivo} ({type(e).__name__}); a continuar em CPU.")
            self._criar_sessao(["CPUExecutionProvider"])
            saidas = self._correr(x)
        h, w = img_bgr.shape[:2]
        return descodificar(saidas, self.cfg, w, h)
