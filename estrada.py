# -*- coding: utf-8 -*-
"""
estrada.py - Deteção da área circulável (estrada) com o YOLOP, em ONNX.

Corre em Windows, Linux e macOS, com ou sem GPU: só precisa de numpy, opencv e
onnxruntime (como o linhas.py). O modelo é o ONNX oficial do YOLOP
(https://github.com/hustvl/YOLOP, licença MIT, weights/yolop-640-640.onnx),
treinado no BDD100K; guardado em modelos/estrada.onnx.

O YOLOP tem três saídas (objetos, área circulável e linhas); aqui usa-se só a área
circulável ("drive_area_seg"): os objetos já são detetados pelo YOLOv8 e as linhas
pelo CLRNet. O pré-processamento replica o do test_onnx.py do YOLOP.
"""

from pathlib import Path

import cv2
import numpy as np

PASTA_MODELOS = Path(__file__).resolve().parent / "modelos"
MEDIA = np.array([0.485, 0.456, 0.406], dtype=np.float32)   # normalização do ImageNet (RGB)
DESVIO = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def preprocessar(img_bgr, lado):
    """Redimensiona mantendo a proporção e completa com cinzento até lado x lado ("letterbox")."""
    h, w = img_bgr.shape[:2]
    r = min(lado / h, lado / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (lado - nw) // 2, (lado - nh) // 2
    tela = np.full((lado, lado, 3), 114, dtype=np.uint8)
    tela[dh:dh + nh, dw:dw + nw] = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    x = (tela[:, :, ::-1].astype(np.float32) / 255.0 - MEDIA) / DESVIO   # BGR -> RGB, normalizar
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None]), (dw, dh, nw, nh)


def desenhar_estrada(img, mascara, cor=(0, 200, 0), opacidade=0.35):
    """Pinta a estrada (mascara == 1) com a cor indicada, semitransparente."""
    if mascara is not None and mascara.any():
        img[mascara] = (img[mascara] * (1 - opacidade) + np.array(cor) * opacidade).astype(np.uint8)
    return img


class DetetorEstrada:
    def __init__(self, caminho_onnx=None, confianca=None):
        import onnxruntime as ort
        import sys
        self.caminho = Path(caminho_onnx) if caminho_onnx else PASTA_MODELOS / "estrada.onnx"
        if not self.caminho.exists():
            raise FileNotFoundError(self.caminho)
        self.ort = ort
        # None: cada píxel fica com a classe mais provável (como no YOLOP). Um valor (ex. 0.9): só é
        # estrada onde a pontuação de "estrada" (0 a 1) é pelo menos esse valor.
        self.confianca = confianca
        if hasattr(ort, "set_default_logger_severity"):
            ort.set_default_logger_severity(3)
        disponiveis = ort.get_available_providers()
        if ("CUDAExecutionProvider" in disponiveis and hasattr(ort, "preload_dlls")
                and "torch" not in sys.modules):
            try:
                ort.preload_dlls()
            except Exception:
                pass
        preferidos = [p for p in ("CUDAExecutionProvider", "DmlExecutionProvider", "CPUExecutionProvider")
                      if p in disponiveis]
        try:
            self._criar_sessao(preferidos)
        except Exception as e:
            print(f"Aviso: não foi possível usar a GPU na deteção da estrada ({type(e).__name__}); a usar o CPU.")
            self._criar_sessao(["CPUExecutionProvider"])

    def _criar_sessao(self, fornecedores):
        opcoes = self.ort.SessionOptions()
        opcoes.log_severity_level = 3
        self.sessao = self.ort.InferenceSession(str(self.caminho), sess_options=opcoes, providers=fornecedores)
        entrada = self.sessao.get_inputs()[0]
        self.entrada, self.lado = entrada.name, int(entrada.shape[2])
        usado = self.sessao.get_providers()[0]
        self.dispositivo = {"CUDAExecutionProvider": "GPU (CUDA)",
                            "DmlExecutionProvider": "GPU (DirectML)"}.get(usado, "CPU")

    def detetar(self, img_bgr):
        """Devolve uma máscara booleana (H, W): True onde há estrada (área circulável)."""
        x, (dw, dh, nw, nh) = preprocessar(img_bgr, self.lado)
        try:
            da = self.sessao.run(["drive_area_seg"], {self.entrada: x})[0]
        except Exception as e:
            if self.dispositivo == "CPU":
                raise
            print(f"\nAviso: a deteção da estrada falhou na {self.dispositivo} ({type(e).__name__}); "
                  "a continuar em CPU.")
            self._criar_sessao(["CPUExecutionProvider"])
            da = self.sessao.run(["drive_area_seg"], {self.entrada: x})[0]
        da = da[0, :, dh:dh + nh, dw:dw + nw]                                # tira o "letterbox"
        # saída: 2 canais (fundo, estrada), cada um com uma pontuação de 0 a 1
        mascara = (da[1] > da[0]) if self.confianca is None else (da[1] >= self.confianca)
        mascara = mascara.astype(np.uint8)
        h, w = img_bgr.shape[:2]
        return cv2.resize(mascara, (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)
