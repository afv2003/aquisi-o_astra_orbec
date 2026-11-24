# -*- coding: utf-8 -*-
"""
objetos.py - Deteção de objetos (YOLOv8n) em ONNX, sem ultralytics nem PyTorch.
"""

import ast
from pathlib import Path

import cv2
import numpy as np

from . import caminhos
from . import gpu

PASTA_MODELOS = caminhos.MODELOS

# Classes detetadas por omissão, no processar_bag.py (offline) e no processar_tempo_real.py (online):
# só elementos de estrada (nomes do COCO, ver README, secção 5). Para mudar a lista de forma
# permanente, nos dois, altere-a aqui (None = as 80 classes do COCO); para uma execução offline,
# use --classes-objetos.
CLASSES_ESTRADA = ["person", "bicycle", "car", "motorcycle", "bus", "truck", "traffic light", "stop sign"]

# Fallback caso os metadados do ONNX não tragam a lista de nomes
NOMES_COCO = {
    0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 4: "airplane", 5: "bus", 6: "train",
    7: "truck", 8: "boat", 9: "traffic light", 10: "fire hydrant", 11: "stop sign",
    12: "parking meter", 13: "bench", 14: "bird", 15: "cat", 16: "dog", 17: "horse",
    18: "sheep", 19: "cow", 20: "elephant", 21: "bear", 22: "zebra", 23: "giraffe",
    24: "backpack", 25: "umbrella", 26: "handbag", 27: "tie", 28: "suitcase", 29: "frisbee",
    30: "skis", 31: "snowboard", 32: "sports ball", 33: "kite", 34: "baseball bat",
    35: "baseball glove", 36: "skateboard", 37: "surfboard", 38: "tennis racket", 39: "bottle",
    40: "wine glass", 41: "cup", 42: "fork", 43: "knife", 44: "spoon", 45: "bowl", 46: "banana",
    47: "apple", 48: "sandwich", 49: "orange", 50: "broccoli", 51: "carrot", 52: "hot dog",
    53: "pizza", 54: "donut", 55: "cake", 56: "chair", 57: "couch", 58: "potted plant",
    59: "bed", 60: "dining table", 61: "toilet", 62: "tv", 63: "laptop", 64: "mouse",
    65: "remote", 66: "keyboard", 67: "cell phone", 68: "microwave", 69: "oven", 70: "toaster",
    71: "sink", 72: "refrigerator", 73: "book", 74: "clock", 75: "vase", 76: "scissors",
    77: "teddy bear", 78: "hair drier", 79: "toothbrush",
}


def preprocessar(img_bgr, lado):
    """Letterbox para lado x lado, BGR -> RGB, [0, 1] e formato NCHW (igual ao YOLOv8)."""
    h, w = img_bgr.shape[:2]
    r = min(lado / h, lado / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (lado - nw) // 2, (lado - nh) // 2
    tela = np.full((lado, lado, 3), 114, dtype=np.uint8)
    tela[dh:dh + nh, dw:dw + nw] = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    x = tela[:, :, ::-1].astype(np.float32) / 255.0
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None]), (r, dw, dh)


class DetetorObjetos:
    def __init__(self, caminho_onnx=None, confianca=0.25, iou=0.7, classes=None):
        import onnxruntime as ort

        self.caminho = Path(caminho_onnx) if caminho_onnx else PASTA_MODELOS / "objetos.onnx"
        if not self.caminho.exists():
            raise FileNotFoundError(self.caminho)

        self.ort = ort
        self.confianca = float(confianca)
        self.iou = float(iou)

        preferidos = gpu.preparar(ort)   # GPU se houver (CUDA ou DirectML), senão CPU (gpu.py)
        try:
            self._criar_sessao(preferidos)
        except Exception as e:
            print(f"Aviso: não foi possível usar a GPU na deteção de objetos ({type(e).__name__}); a usar o CPU.")
            self._criar_sessao(["CPUExecutionProvider"])
        self.dispositivo = gpu.confirmar(self.sessao, "objetos")   # a GPU pode só falhar ao correr

        # Lê os nomes das classes gravados pelo Ultralytics nos metadados do ONNX
        meta = self.sessao.get_modelmeta().custom_metadata_map
        self.nomes = ast.literal_eval(meta["names"]) if "names" in meta else dict(NOMES_COCO)

        # Filtro opcional de classes (aceita lista de nomes ou None para todas)
        if classes is None:
            self.classes_ids = None
        else:
            por_nome = {nome: i for i, nome in self.nomes.items()}
            desconhecidas = [c for c in classes if c not in por_nome]
            if desconhecidas or not classes:
                raise ValueError(
                    f"Classe(s) desconhecida(s): {', '.join(desconhecidas) or '(nenhuma)'}. "
                    f"Possíveis: {', '.join(self.nomes.values())}"
                )
            self.classes_ids = sorted(por_nome[c] for c in classes)

    def _criar_sessao(self, fornecedores):
        opcoes = self.ort.SessionOptions()
        opcoes.log_severity_level = 3
        self.sessao = self.ort.InferenceSession(str(self.caminho), sess_options=opcoes, providers=fornecedores)
        entrada = self.sessao.get_inputs()[0]
        self.entrada = entrada.name
        self.lado = int(entrada.shape[2])
        self.dispositivo = gpu.dispositivo(self.sessao)

    def detetar(self, img_bgr):
        """Devolve [(x1, y1, x2, y2, nome_classe, confianca), ...]."""
        h, w = img_bgr.shape[:2]
        x, (r, dw, dh) = preprocessar(img_bgr, self.lado)
        try:
            saida = self.sessao.run(None, {self.entrada: x})[0]
        except Exception as e:
            if self.dispositivo == "CPU":
                raise
            print(f"\nAviso: a deteção de objetos falhou na {self.dispositivo} ({type(e).__name__}); "
                  "a continuar em CPU.")
            self._criar_sessao(["CPUExecutionProvider"])
            saida = self.sessao.run(None, {self.entrada: x})[0]

        # Saída do YOLOv8: (1, 4 + num_classes, 8400) -> (8400, 4 + num_classes)
        pred = saida[0].T
        pontuacoes = pred[:, 4:]
        cls_ids = pontuacoes.argmax(axis=1)
        confs = pontuacoes[np.arange(len(pred)), cls_ids]

        mascara = confs >= self.confianca
        if self.classes_ids is not None:
            mascara &= np.isin(cls_ids, self.classes_ids)
        if not mascara.any():
            return []

        caixas = pred[mascara, :4]
        cls_ids = cls_ids[mascara]
        confs = confs[mascara]

        cx, cy, bw, bh = caixas[:, 0], caixas[:, 1], caixas[:, 2], caixas[:, 3]
        x1 = np.clip((cx - bw / 2 - dw) / r, 0, w)
        y1 = np.clip((cy - bh / 2 - dh) / r, 0, h)
        x2 = np.clip((cx + bw / 2 - dw) / r, 0, w)
        y2 = np.clip((cy + bh / 2 - dh) / r, 0, h)

        # NMS separado por classe (deslocamento por classe, igual ao Ultralytics)
        desloc = cls_ids * max(w, h)
        caixas_nms = np.stack([x1 + desloc, y1 + desloc, x2 - x1, y2 - y1], axis=1).tolist()
        indices = cv2.dnn.NMSBoxes(caixas_nms, confs.tolist(), self.confianca, self.iou)
        if len(indices) == 0:
            return []

        return [
            (float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]),
             self.nomes[int(cls_ids[i])], float(confs[i]))
            for i in np.asarray(indices).flatten()
        ]
