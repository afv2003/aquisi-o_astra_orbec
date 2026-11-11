# -*- coding: utf-8 -*-
"""
linhas.py - Deteção de linhas da estrada com CLRNet / CLRerNet exportado para ONNX.

Corre em Windows, Linux e macOS, com ou sem GPU: só precisa de numpy, opencv e
onnxruntime (não precisa do UnLanedet, nem de PyTorch, nem de compilar CUDA).

O modelo (modelos/linhas.onnx) e a sua configuração (modelos/linhas.json) são
gerados uma única vez com o exportar_linhas.py, a partir dos pesos do UnLanedet.

O pós-processamento (filtrar por confiança, NMS de linhas e conversão em pontos)
replica o do UnLanedet (unlanedet/model/CLRNet/clr_head.py e o NMS em CUDA de
unlanedet/layers/ops/csrc/nms_kernel.cu), escrito em numpy.
"""

import json
from pathlib import Path

import cv2
import numpy as np

PASTA_MODELOS = Path(__file__).resolve().parent / "modelos"

# Valores por omissão = configuração CULane do UnLanedet (config/clrnet/resnet34_culane.py)
CONFIG_OMISSAO = {
    "img_w": 800,               # largura da imagem de entrada do modelo
    "img_h": 320,               # altura da imagem de entrada do modelo
    "n_offsets": 72,            # pontos por linha (num_points)
    "conf_threshold": 0.4,      # confiança mínima de uma linha
    "nms_thres": 50,            # distância média (px, na imagem 800x320) para duas linhas serem "a mesma"
    "max_lanes": 4,             # máximo de linhas por imagem
    "corte_topo": 270 / 590,    # fração da imagem cortada em cima (no CULane: 270 de 590 px)
}

CORES = [(0, 255, 255), (255, 0, 255), (0, 165, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]


# --------------------------------------------------------------------------- #
# Pós-processamento (independente do onnxruntime: também usado pelo exportar_linhas.py)
# --------------------------------------------------------------------------- #
def _scores(pred):
    """Probabilidade de cada proposta ser uma linha (softmax das 2 primeiras colunas)."""
    logits = pred[:, :2].astype(np.float64)
    e = np.exp(logits - logits.max(axis=1, keepdims=True))
    return e[:, 1] / e.sum(axis=1)


def _sobrepoe(a, b, n_strips, n_offsets, limiar):
    """devIoU do nms_kernel.cu: a e b no formato [l0, l1, start_y, start_x, comprimento, xs...]."""
    start_a = int(a[2] * n_strips + 0.5)
    start_b = int(b[2] * n_strips + 0.5)
    start = max(start_a, start_b)
    end_a = int(start_a + a[4] - 1 + 0.5 - (1 if (a[4] - 1) < 0 else 0))
    end_b = int(start_b + b[4] - 1 + 0.5 - (1 if (b[4] - 1) < 0 else 0))
    end = min(end_a, end_b, n_offsets - 1)
    if end < start:
        return False
    dist = np.abs(a[5 + start:5 + end + 1] - b[5 + start:5 + end + 1]).sum()
    return dist < limiar * (end - start + 1)


def nms_linhas(pred, scores, limiar, top_k, img_w, n_offsets):
    """NMS de linhas (igual ao do UnLanedet). Devolve os índices das linhas mantidas."""
    n_strips = n_offsets - 1
    caixas = np.concatenate([pred[:, :4], pred[:, 5:]], axis=1).astype(np.float64)
    caixas[:, 4] = caixas[:, 4] * n_strips           # comprimento em "faixas"
    caixas[:, 5:] = caixas[:, 5:] * (img_w - 1)      # x em píxeis da imagem do modelo
    ordem = np.argsort(-scores, kind="stable")
    removida = np.zeros(len(ordem), dtype=bool)
    manter = []
    for oi, i in enumerate(ordem):
        if removida[oi]:
            continue
        manter.append(int(i))
        if len(manter) == top_k:
            break
        for oj in range(oi + 1, len(ordem)):
            if not removida[oj] and _sobrepoe(caixas[i], caixas[ordem[oj]], n_strips, n_offsets, limiar):
                removida[oj] = True
    return manter


def descodificar(saida, cfg, largura, altura, corte_px):
    """
    Converte a saída do modelo (192 x 78) em linhas, cada uma um array (N, 2) de
    pontos (x, y) em píxeis da imagem original (largura x altura).
    """
    n_offsets = cfg["n_offsets"]
    n_strips = n_offsets - 1
    prior_ys = np.linspace(1, 0, n_offsets)
    pred = np.asarray(saida, dtype=np.float64).reshape(-1, 6 + n_offsets)

    scores = _scores(pred)
    pred, scores = pred[scores >= cfg["conf_threshold"]], scores[scores >= cfg["conf_threshold"]]
    if len(pred) == 0:
        return []
    pred = pred[nms_linhas(pred, scores, cfg["nms_thres"], cfg["max_lanes"], cfg["img_w"], n_offsets)]

    linhas = []
    for p in pred:
        xs = p[6:].copy()                                   # x normalizados (0..1)
        start = min(max(0, int(round(p[2] * n_strips))), n_strips)
        comprimento = int(round(p[5] * n_strips))
        end = min(start + comprimento - 1, n_offsets - 1)
        # abaixo do início, só se prolonga a linha enquanto se mantiver dentro da imagem
        dentro = ((xs[:start] >= 0.0) & (xs[:start] <= 1.0))[::-1].cumprod()[::-1].astype(bool)
        xs[end + 1:] = -2
        xs[:start][~dentro] = -2
        validos = xs >= 0
        lx, ly = xs[validos][::-1], prior_ys[validos][::-1]
        if len(lx) <= 1:
            continue
        px = lx * largura
        py = ly * (altura - corte_px) + corte_px
        ok = (px >= 0) & (px < largura)
        if ok.sum() > 1:
            linhas.append(np.stack([px[ok], py[ok]], axis=1))
    return linhas


def preprocessar(img_bgr, cfg, corte_topo=None):
    """Imagem BGR (H, W, 3) -> tensor (1, 3, img_h, img_w) float32, como no UnLanedet."""
    h = img_bgr.shape[0]
    corte_px = int(round(h * (cfg["corte_topo"] if corte_topo is None else corte_topo)))
    recorte = img_bgr[corte_px:, :, :]
    redim = cv2.resize(recorte, (cfg["img_w"], cfg["img_h"]), interpolation=cv2.INTER_CUBIC)
    x = (redim.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
    return np.ascontiguousarray(x), corte_px


def desenhar_linhas(img, linhas, espessura=3):
    for i, pts in enumerate(linhas):
        cor = CORES[i % len(CORES)]
        cv2.polylines(img, [np.round(pts).astype(np.int32).reshape(-1, 1, 2)], False, cor, espessura,
                      cv2.LINE_AA)
    return img


def carregar_config(caminho_json=None):
    cfg = dict(CONFIG_OMISSAO)
    caminho_json = Path(caminho_json) if caminho_json else PASTA_MODELOS / "linhas.json"
    if caminho_json.exists():
        cfg.update(json.loads(caminho_json.read_text(encoding="utf-8")))
    return cfg


# --------------------------------------------------------------------------- #
# Detetor (onnxruntime)
# --------------------------------------------------------------------------- #
class DetetorLinhas:
    def __init__(self, caminho_onnx=None, caminho_json=None, corte_topo=None):
        import onnxruntime as ort
        self.caminho = Path(caminho_onnx) if caminho_onnx else PASTA_MODELOS / "linhas.onnx"
        if not self.caminho.exists():
            raise FileNotFoundError(self.caminho)
        self.cfg = carregar_config(caminho_json or self.caminho.with_suffix(".json"))
        if corte_topo is not None:
            self.cfg["corte_topo"] = corte_topo
        # GPU se houver: CUDA (onnxruntime-gpu, NVIDIA) ou DirectML (onnxruntime-directml, Windows);
        # se não houver, ou se falhar, usa o CPU.
        self.ort = ort
        if hasattr(ort, "set_default_logger_severity"):
            ort.set_default_logger_severity(3)  # avisos do CUDA (ScatterND) saem no registo geral, não no da sessão
        disponiveis = ort.get_available_providers()
        import sys
        if ("CUDAExecutionProvider" in disponiveis and hasattr(ort, "preload_dlls")
                and "torch" not in sys.modules):  # com o PyTorch carregado, o CUDA já está carregado
            try:  # carrega as bibliotecas CUDA/cuDNN instaladas pelo pip (sem instalar o CUDA no sistema)
                ort.preload_dlls()
            except Exception:
                pass
        preferidos = [p for p in ("CUDAExecutionProvider", "DmlExecutionProvider", "CPUExecutionProvider")
                      if p in disponiveis]
        try:
            self._criar_sessao(preferidos)
        except Exception as e:
            print(f"Aviso: não foi possível usar a GPU na deteção de linhas ({type(e).__name__}); a usar o CPU.")
            self._criar_sessao(["CPUExecutionProvider"])

    def _criar_sessao(self, fornecedores):
        opcoes = self.ort.SessionOptions()
        opcoes.log_severity_level = 3   # só erros (esconde avisos inofensivos, ex. ScatterND/Memcpy)
        self.sessao = self.ort.InferenceSession(str(self.caminho), sess_options=opcoes, providers=fornecedores)
        self.entrada = self.sessao.get_inputs()[0].name
        usado = self.sessao.get_providers()[0]
        self.dispositivo = {"CUDAExecutionProvider": "GPU (CUDA)",
                            "DmlExecutionProvider": "GPU (DirectML)"}.get(usado, "CPU")

    def detetar(self, img_bgr):
        """Devolve uma lista de linhas; cada linha é um array (N, 2) de pontos (x, y) em píxeis."""
        x, corte_px = preprocessar(img_bgr, self.cfg)
        try:
            saida = self.sessao.run(None, {self.entrada: x})[0]
        except Exception as e:
            if self.dispositivo == "CPU":
                raise
            print(f"\nAviso: a deteção de linhas falhou na {self.dispositivo} ({type(e).__name__}); "
                  "a continuar em CPU.")
            self._criar_sessao(["CPUExecutionProvider"])
            saida = self.sessao.run(None, {self.entrada: x})[0]
        h, w = img_bgr.shape[:2]
        return descodificar(saida[0], self.cfg, w, h, corte_px)
