# -*- coding: utf-8 -*-
"""
Os três modelos ONNX estão na imagem (src/va_g05/modelos) e correm com o onnxruntime da imagem
(CPU no CI; GPU, se houver, no carro): uma imagem sintética passa pelos três detetores sem erros e
com as saídas no formato que o processamento espera.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

try:
    import va_g05  # noqa: F401
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("onnxruntime")
from va_g05 import caminhos, estrada, linhas, objetos  # noqa: E402

IMAGEM = np.full((480, 640, 3), 90, np.uint8)
IMAGEM[300:, :] = 60                                    # "estrada" escura em baixo


def test_ficheiros_dos_modelos():
    for nome in ("objetos.onnx", "linhas.onnx", "linhas.json", "estrada.onnx"):
        assert (caminhos.MODELOS / nome).is_file(), nome


def test_objetos():
    caixas = objetos.DetetorObjetos().detetar(IMAGEM)
    assert isinstance(caixas, list)


def test_linhas():
    assert isinstance(linhas.DetetorLinhas().detetar(IMAGEM), list)


def test_estrada():
    mascara = estrada.DetetorEstrada().detetar(IMAGEM)
    assert mascara.shape[:2] == IMAGEM.shape[:2]
