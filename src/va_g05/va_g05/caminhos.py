# -*- coding: utf-8 -*-
"""
caminhos.py - Onde estão a configuração, os modelos e os dados, no Docker e fora dele.

O pacote va_g05 é usado de três maneiras, e os ficheiros estão em sítios diferentes em cada uma:
  * Docker (online): o código está instalado em /ws/install (colcon build); a configuração e os
    modelos ficam na pasta do pacote, /ws/src/va_g05 (variável VA_G05_DIR, definida no Dockerfile);
    os dados (gravações, bags) em /data, a pasta ./data do repositório (ou o disco externo, DADOS no .env).
  * Offline e ferramentas (venv, Windows ou Linux): offline/*.py e tools/*.py usam o código-fonte
    em src/va_g05; a configuração e os modelos estão ao lado (src/va_g05/config, src/va_g05/modelos);
    os dados em <repositório>/data.
  * ros2 run sem VA_G05_DIR (fora do Docker): a configuração instalada (share/va_g05/config).
"""
import os
from pathlib import Path


def _pasta_pacote():
    if os.environ.get("VA_G05_DIR"):
        return Path(os.environ["VA_G05_DIR"]).resolve()
    fonte = Path(__file__).resolve().parents[1]            # src/va_g05 (código-fonte)
    if (fonte / "config").is_dir():
        return fonte
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("va_g05"))
    except Exception:
        return fonte


PACOTE = _pasta_pacote()
CONFIG = PACOTE / "config"           # calibração, montagem das câmaras, parâmetros, RViz
MODELOS = PACOTE / "modelos"         # objetos.onnx, linhas.onnx (+ .json), estrada.onnx
RAIZ = PACOTE.parents[1]             # raiz do repositório (fora do Docker)


def _pasta_dados():
    if os.environ.get("PASTA_DADOS"):
        return Path(os.environ["PASTA_DADOS"])
    if Path("/data").is_dir() and not (RAIZ / "data").is_dir():
        return Path("/data")                                  # dentro do Docker
    return RAIZ / "data"


DADOS = _pasta_dados()               # gravações do online: data/<sessão>/
OFFLINE = DADOS / "offline"          # registos do offline: data/offline/<viagem>/<ensaio>/


def calibracao(cam=None):
    """Calibração de uma câmara (formato ROS camera_info): config/camera_astra_<N>.yaml."""
    return CONFIG / (f"camera_astra_{cam}.yaml" if cam else "camera_astra.yaml")
