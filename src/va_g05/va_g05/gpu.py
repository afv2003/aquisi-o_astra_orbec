# -*- coding: utf-8 -*-
"""
gpu.py - Escolha do dispositivo (GPU ou CPU) dos modelos ONNX, comum a todos os detetores
(objetos.py, linhas.py, estrada.py).

1. preparar(ort), antes de criar a sessão: devolve os fornecedores por ordem de preferência
   (CUDA, DirectML, CPU) e prepara as bibliotecas do CUDA/cuDNN instaladas pelo pip.
   Com o onnxruntime-gpu[cuda,cudnn], essas bibliotecas ficam no venv (site-packages/nvidia/*/bin).
   O ort.preload_dlls() carrega as principais, mas o cuDNN carrega outras sozinho mais tarde (por
   exemplo o cudnn_engines_tensor_ir64_9.dll das versões recentes) e, no Windows, só as encontra se
   essas pastas estiverem no PATH. Sem isso, a primeira convolução falha ("Could not locate
   cudnn_..."), o onnxruntime passa sozinho para o CPU e tudo fica várias vezes mais lento.
   Por isso, no Windows, essas pastas são acrescentadas ao PATH (só deste processo).
2. confirmar(sessao, nome), logo depois de criar a sessão: corre o modelo uma vez com uma entrada
   neutra. A GPU pode falhar só ao correr e não ao criar a sessão; nesse caso o onnxruntime passa
   para o CPU sem dar erro. Devolve o dispositivo que ficou de facto (o que aparece no ecrã e no
   registo) e avisa se não foi a GPU. Também tira a demora da 1.ª deteção dos tempos medidos.
"""
import os
import sys
from pathlib import Path

NOMES = {"CUDAExecutionProvider": "GPU (CUDA)", "DmlExecutionProvider": "GPU (DirectML)"}
OPCOES_CUDA_MEMORIA_REDUZIDA = {
    "arena_extend_strategy": "kSameAsRequested",
    "cudnn_conv_algo_search": "HEURISTIC",
    "cudnn_conv_use_max_workspace": "0",
}
_preparado = False
_pastas_dll = []          # referências do os.add_dll_directory (as pastas saem da procura se forem libertadas)


def _pastas_nvidia():
    """Pastas bin das bibliotecas NVIDIA instaladas pelo pip (site-packages/nvidia/<biblioteca>/bin)."""
    pastas = []
    for sp in sys.path:
        base = Path(sp or ".") / "nvidia"
        if base.is_dir():
            pastas += [p for p in sorted(base.glob("*/bin")) if p.is_dir() and p not in pastas]
    return pastas


def preparar(ort):
    """Fornecedores (GPU primeiro, CPU no fim) e, na primeira vez, as bibliotecas do CUDA/cuDNN do pip."""
    global _preparado
    if hasattr(ort, "set_default_logger_severity"):
        ort.set_default_logger_severity(3)   # avisos inofensivos do CUDA (ScatterND/Memcpy) ficam escondidos
    if os.environ.get("USAR_GPU") == "0":
        return ["CPUExecutionProvider"]

    disponiveis = ort.get_available_providers()
    # com o PyTorch carregado, o CUDA já está carregado (não se mexe)
    if not _preparado and "CUDAExecutionProvider" in disponiveis and "torch" not in sys.modules:
        _preparado = True
        if os.name == "nt":
            pastas = [str(p) for p in _pastas_nvidia()]
            if pastas:
                os.environ["PATH"] = os.pathsep.join(pastas + [os.environ.get("PATH", "")])
                for p in pastas:
                    try:
                        _pastas_dll.append(os.add_dll_directory(p))
                    except OSError:
                        pass
        if hasattr(ort, "preload_dlls"):
            try:
                ort.preload_dlls()
            except Exception:
                pass
    fornecedores = [p for p in ("CUDAExecutionProvider", "DmlExecutionProvider", "CPUExecutionProvider")
                    if p in disponiveis]
    if "CUDAExecutionProvider" in fornecedores and os.environ.get("CUDA_MEMORIA_REDUZIDA") == "1":
        fornecedores[0] = ("CUDAExecutionProvider", OPCOES_CUDA_MEMORIA_REDUZIDA)
    return fornecedores


def dispositivo(sessao):
    return NOMES.get(sessao.get_providers()[0], "CPU")


def confirmar(sessao, nome):
    """Corre o modelo uma vez com uma entrada neutra e devolve o dispositivo que ficou em uso."""
    import numpy as np
    antes = dispositivo(sessao)
    if antes == "CPU":
        return antes
    entradas = {}
    for e in sessao.get_inputs():
        forma = [d if isinstance(d, int) and d > 0 else (1 if i == 0 else 640) for i, d in enumerate(e.shape)]
        tipo = np.float16 if "float16" in e.type else np.float32
        entradas[e.name] = np.zeros(forma, dtype=tipo)
    try:
        sessao.run(None, entradas)
    except Exception:
        pass               # se falhar de outra forma, a deteção trata disso (passa para o CPU)
    depois = dispositivo(sessao)
    if depois != antes:
        print(f"Aviso: a {antes} falhou ao correr o modelo de {nome}; a usar o CPU (mais lento). "
              "Ver README, secção 4.5 (offline) ou 3.3 (Docker).")
    return depois
