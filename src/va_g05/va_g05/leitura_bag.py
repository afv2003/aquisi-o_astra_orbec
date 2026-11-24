# -*- coding: utf-8 -*-
"""
leitura_bag.py - Leitura dos ROS bags (.mcap) e correção da distorção.

Usado pelo processar_bag.py (processamento) e pelo reproduzir_registo.py (vídeo a partir de um
registo de deteções), para os dois lerem os bags e corrigirem a imagem exatamente da mesma forma.
"""
import re
from pathlib import Path

import cv2
import numpy as np
from mcap.reader import make_reader, NonSeekingReader
from mcap_ros2.decoder import DecoderFactory
from mcap_ros2.reader import read_ros2_messages

MAGIA_MCAP = b"\x89MCAP0\r\n"   # um ficheiro MCAP começa sempre com estes 8 bytes


def verificar_mcap(caminho):
    """Devolve None se o ficheiro parece um MCAP; senão, uma mensagem de erro."""
    caminho = Path(caminho)
    if not caminho.exists():
        return f"Ficheiro '{caminho}' não encontrado."
    with open(caminho, "rb") as f:
        inicio = f.read(8)
    if inicio != MAGIA_MCAP:
        tamanho = caminho.stat().st_size
        msg = f"'{caminho}' não é um ficheiro MCAP válido ({tamanho} bytes)."
        if tamanho < 1024:
            msg += " O ficheiro está vazio: a gravação não gravou nada, ou a cópia falhou."
        return msg
    return None


def bag_tem_indice(caminho):
    """
    Um MCAP tem no fim um índice (resumo), escrito quando a gravação termina corretamente.
    Se a gravação foi interrompida (ou a cópia ficou incompleta), esse índice falta ou está estragado.
    """
    try:
        with open(caminho, "rb") as f:
            make_reader(f).get_summary()
        return True
    except Exception:
        return False


def topicos_do_bag(caminho):
    try:
        with open(caminho, "rb") as f:
            resumo = make_reader(f).get_summary()
        return sorted({c.topic for c in resumo.channels.values()}) if resumo else []
    except Exception:
        return []


def intervalo_do_bag(caminho):
    """(primeiro, último) instante das mensagens, em ns, lido do índice; (None, None) se não houver."""
    try:
        with open(caminho, "rb") as f:
            resumo = make_reader(f).get_summary()
        if resumo and resumo.statistics:
            return resumo.statistics.message_start_time, resumo.statistics.message_end_time
    except Exception:
        pass
    return None, None


def ler_mensagens(caminho, topicos, indice_ok, inicio_ns=None, fim_ns=None):
    """
    Mensagens dos tópicos indicados, em ordem de tempo (opcionalmente só entre inicio_ns e fim_ns,
    inclusive).
    * Bag com índice: os dados estão em blocos ("chunks"), cada um com um código de verificação
      (CRC). Um bloco estragado (pontos coloridos na imagem, ou erro de leitura) é saltado e a
      leitura continua no bloco seguinte.
    * Bag sem índice (cortado no fim): lido sequencialmente até ao ponto onde foi cortado.
    """
    lidas = 0
    if not indice_ok:
        with open(caminho, "rb") as f:
            leitor = NonSeekingReader(f, decoder_factories=[DecoderFactory()])
            try:
                for m in read_ros2_messages(leitor, topics=topicos, log_time_order=False):
                    if inicio_ns is not None and m.log_time_ns < inicio_ns:
                        continue
                    if fim_ns is not None and m.log_time_ns > fim_ns:
                        break
                    lidas += 1
                    yield m
            except Exception as e:
                print(f"\nAviso: o bag está cortado ou com dados estragados a partir deste ponto "
                      f"({type(e).__name__}); foram lidas {lidas} mensagens.")
        return

    fim_excl = None if fim_ns is None else fim_ns + 1
    with open(caminho, "rb") as f:
        leitor = make_reader(f, validate_crcs=True, decoder_factories=[DecoderFactory()])
        resumo = leitor.get_summary()
        canais = {cid for cid, c in resumo.channels.items() if c.topic in topicos}
        blocos = sorted((c for c in resumo.chunk_indexes
                         if not c.message_index_offsets or canais & set(c.message_index_offsets)),
                        key=lambda c: (c.message_start_time, c.chunk_start_offset))
        inicio, ultimo = inicio_ns, -1
        saltados, perdido_s = 0, 0.0
        while True:
            try:
                for m in read_ros2_messages(leitor, topics=topicos, start_time=inicio, end_time=fim_excl):
                    lidas += 1
                    ultimo = m.log_time_ns
                    yield m
                break
            except Exception as e:
                # o bloco estragado é o primeiro que ainda não foi lido; continua-se depois dele
                limite = max(ultimo, (inicio or 0) - 1)
                restantes = [c for c in blocos if c.message_end_time > limite]
                if not restantes or (fim_ns is not None and restantes[0].message_start_time > fim_ns):
                    print(f"\nAviso: dados estragados no fim do bag ({type(e).__name__}); "
                          f"foram lidas {lidas} mensagens.")
                    break
                mau = restantes[0]
                inicio = mau.message_end_time + 1
                saltados += 1
                perdido_s += max(0.0, (mau.message_end_time - max(mau.message_start_time, limite)) / 1e9)
                print(f"\nAviso: bloco estragado no bag ({type(e).__name__}); "
                      f"saltado (cerca de {perdido_s:.1f} s perdidos até agora, {saltados} bloco(s)).")
        if saltados:
            print(f"\nBag com {saltados} bloco(s) estragado(s), saltado(s): cerca de {perdido_s:.1f} s "
                  f"de vídeo em falta. O ficheiro foi danificado depois de gravado (cópia ou disco).")


def imagem_bgr(ros_msg):
    """sensor_msgs/Image ou sensor_msgs/CompressedImage (JPEG/PNG) -> imagem BGR (numpy), ou None se a
    codificação não for suportada."""
    if not hasattr(ros_msg, "encoding"):               # CompressedImage (bags do online, desde a compressão)
        img = cv2.imdecode(np.frombuffer(ros_msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        # formato do compressed_image_transport: "<original>; jpeg compressed <codificação comprimida>";
        # o JPEG vem quase sempre em bgr8 (o imdecode devolve BGR); só em rgb8 é preciso trocar as cores
        formato = (getattr(ros_msg, "format", "") or "").strip()
        if img is not None and formato.endswith("compressed rgb8"):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    w, h = ros_msg.width, ros_msg.height
    if ros_msg.encoding == 'yuv422_yuy2':
        yuv = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape((h, w, 2))
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
    if ros_msg.encoding in ('rgb8', 'bgr8'):
        img = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape((h, w, 3)).copy()
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if ros_msg.encoding == 'rgb8' else img
    return None


# --------------------------------------------------------------------------- #
# Câmara e tópico
# --------------------------------------------------------------------------- #
TOPICO_TRECHOS = "/processamento/trechos"


def topico_imagem(camara):
    """Tópico das imagens de uma câmara num bag (formato atual, o do online: JPEG)."""
    return f"/camara{camara}/image_raw/compressed"


def camaras_no_bag(topicos):
    """{'5': '/camara5/image_raw/compressed', ...} a partir dos tópicos do bag (formato atual: imagens
    em JPEG, um tópico por câmara)."""
    cams = {}
    for t in sorted(topicos):
        m = re.fullmatch(r"/camara([0-9A-Za-z]+)/image_raw/compressed", t)
        if m:
            cams[m.group(1)] = t
    return cams


def formato_atual(topicos):
    """None se o bag está no formato atual (o do online: imagens em /camara<N>/image_raw/compressed e
    os trechos em /processamento/trechos); senão, o texto do erro."""
    if camaras_no_bag(topicos) and TOPICO_TRECHOS in topicos:
        return None
    return ("o bag não está no formato atual (imagens em /camara<N>/image_raw/compressed e trechos em "
            "/processamento/trechos, como grava o online). Um bag antigo converte-se uma vez com: "
            "python tools/converter_bag_antigo.py <bag> (README, secção 3.1)")


# --------------------------------------------------------------------------- #
# Calibração (correção da distorção da lente)
# --------------------------------------------------------------------------- #
def carregar_calibracao_yaml(caminho):
    """Lê um camera_astra.yaml (formato ROS camera_info) -> (K, D, nova_K, (largura, altura))."""
    import yaml
    with open(caminho, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    K = np.array(c["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"], dtype=np.float64)
    P = np.array(c["projection_matrix"]["data"], dtype=np.float64).reshape(3, 4)
    return K, D, P[:, :3], (int(c["image_width"]), int(c["image_height"]))


def carregar_calibracao_bag(caminho_mcap, camara):
    """Alternativa: lê o 1.º /camera_info gravado no bag (se o usb_cam tiver calibração)."""
    try:
        for m in read_ros2_messages(str(caminho_mcap),
                                    topics=['/camera_info'] + ([f'/camara{camara}/camera_info'] if camara else [])):
            ci = m.ros_msg
            K = np.array(ci.k, dtype=np.float64).reshape(3, 3)
            if K[0, 0] == 0:  # usb_cam sem calibração publica zeros
                return None
            P = np.array(ci.p, dtype=np.float64).reshape(3, 4)
            return K, np.array(ci.d, dtype=np.float64), P[:, :3], (ci.width, ci.height)
    except Exception:
        pass
    return None


def calibracao_para_dict(calibracao, origem):
    """Para guardar a calibração usada no registo (ensaio.json)."""
    K, D, nova_K, (w, h) = calibracao
    return {"origem": origem, "K": K.tolist(), "D": np.asarray(D).ravel().tolist(),
            "nova_K": nova_K.tolist(), "tamanho": [int(w), int(h)]}


def calibracao_de_dict(d):
    return (np.array(d["K"], dtype=np.float64), np.array(d["D"], dtype=np.float64),
            np.array(d["nova_K"], dtype=np.float64), tuple(d["tamanho"]))


class CorretorDistorcao:
    """Remove a distorção da lente. Os mapas são calculados uma única vez por resolução."""

    def __init__(self, calibracao):
        self.calibracao = calibracao
        self._mapas = {}

    def __call__(self, img):
        h, w = img.shape[:2]
        if (w, h) not in self._mapas:
            K, D, nova_K, (wc, hc) = self.calibracao
            if (w, h) != (wc, hc):
                print(f"Aviso: calibração feita em {wc}x{hc} mas o bag é {w}x{h}; a escalar K.")
                s = np.diag([w / wc, h / hc, 1.0])
                K, nova_K = s @ K, s @ nova_K
            self._mapas[(w, h)] = cv2.initUndistortRectifyMap(K, D, None, nova_K, (w, h), cv2.CV_16SC2)
        m1, m2 = self._mapas[(w, h)]
        return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
