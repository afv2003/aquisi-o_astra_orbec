# -*- coding: utf-8 -*-
"""
visualizador.py - Nó de visualização do online, dentro do Docker (serviço "visualizador").

O guião da avaliação recomenda separar o processamento da representação visual: um nó recebe os dados
dos sensores e publica a informação processada (processar_tempo_real.py), outro nó subscreve essa
informação e mostra-a. Este é o segundo: não corre nenhum modelo, só subscreve tópicos e desenha.
Serve igual ao vivo (câmaras) e em reprodução (reprodutor.py), e desenha com o painel.py, o mesmo
do offline (processar_bag.py, reproduzir_registo.py e GUI): o ecrã é igual nos dois.

Subscreve:
  /camara<N>/image_raw                          imagens (as mesmas que o processamento recebe)
  /camara<N>/processamento/dados_deteccoes      deteções de cada imagem (JSON, com o instante dela)
  /processamento/painel                         tempos dos modelos, deteções/s, gravação, correção
  /processamento/estado_camaras                 câmaras ligadas / desligadas (tecla D)
  /<grupo>/grelha_local, /<grupo>/aviso_faixa   grelha local e aviso de mudança de faixa
  /<grupo>/estado                               estado do planeador A* comum
  /processamento/reproducao                     estado do reprodutor (só em reprodução)
Cada deteção é desenhada sobre a sua imagem (a do mesmo instante, header.stamp), corrigida da
distorção como no processamento (config/camera_astra_<N>.yaml).

Publica:
  /processamento/comando                        teclas: R ("trecho"), D ("alternar_direita"),
                                                Espaço ("pausa", em reprodução), Q / Esc ("terminar")
  /camara<N>/processamento/imagem_anotada       a imagem de cada câmara com os desenhos (só se alguém
                                                a subscrever, por exemplo o RViz ou o Foxglove)

Variáveis de ambiente: CAMARAS_ATIVAS ("5 8", "5" ou "8"), SEM_CORRECAO (como no processamento).
"""
import array
import json
import os
import signal
import sys
import threading
import time
from collections import OrderedDict, deque
from functools import partial
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
import visual
import painel
import leitura_bag
import registo
import gravacao_online   # só para ler o instante das imagens (sem as descodificar)
import geometria
import grelha_local

JANELA = "Online - G5"
TOPICO_ESTADO = "/processamento/estado_camaras"
TOPICO_COMANDO = "/processamento/comando"
TOPICO_PAINEL = "/processamento/painel"
TOPICO_REPRODUCAO = "/processamento/reproducao"
QOS_ESTADO = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MAX_IMAGENS = 120          # imagens guardadas por câmara à espera das deteções (cerca de 4 s a 30 img/s)


class Visualizador(Node):
    def __init__(self, camaras, grupo):
        super().__init__("visualizador")
        self.lock = threading.Lock()
        self.imagens = {c: OrderedDict() for c in camaras}      # {câmara: {stamp_ns: bytes}}
        self.dados = {c: deque(maxlen=10) for c in camaras}    # deteções ainda por desenhar
        self.painel = {}
        self.estado = {}
        self.grelha = None
        self.aviso = None
        self.planeador = None
        self.reproducao = {}
        self.pubs_anotada = {}
        for c in camaras:
            self.create_subscription(ROSImage, f"/camara{c}/image_raw", partial(self.cb_imagem, cam=c),
                                     qos_profile_sensor_data, raw=True)
            self.create_subscription(String, registo.topico_deteccoes(c), partial(self.cb_dados, cam=c), 10)
            self.pubs_anotada[c] = self.create_publisher(ROSImage, f"/camara{c}/processamento/imagem_anotada", 10)
        self.create_subscription(String, TOPICO_PAINEL, self.cb_painel, 10)
        self.create_subscription(String, TOPICO_ESTADO, self.cb_estado, QOS_ESTADO)
        self.create_subscription(String, TOPICO_REPRODUCAO, self.cb_reproducao, QOS_ESTADO)
        self.create_subscription(String, f"/{grupo}/aviso_faixa", self.cb_aviso, 10)
        self.create_subscription(String, f"/{grupo}/estado", self.cb_planeador, 10)
        try:
            from nav_msgs.msg import OccupancyGrid
            self.create_subscription(OccupancyGrid, f"/{grupo}/grelha_local", self.cb_grelha, 10)
        except ImportError:
            pass
        self.pub_comando = self.create_publisher(String, TOPICO_COMANDO, 10)

    # --------------------------------------------------------------- receção
    def cb_imagem(self, dados, cam):
        t = gravacao_online.stamp_da_imagem(dados)
        with self.lock:
            buf = self.imagens[cam]
            buf[t] = dados
            while len(buf) > MAX_IMAGENS:
                buf.popitem(last=False)

    def cb_dados(self, msg, cam):
        try:
            self.dados[cam].append(json.loads(msg.data))
        except ValueError:
            pass

    @staticmethod
    def _json(msg):
        try:
            return json.loads(msg.data)
        except ValueError:
            return None

    def cb_painel(self, msg):
        p = self._json(msg)
        if p and p.get("camara") is not None:
            self.painel[str(p["camara"])] = p

    def cb_estado(self, msg):
        self.estado = self._json(msg) or {}

    def cb_reproducao(self, msg):
        self.reproducao = self._json(msg) or {}

    def cb_aviso(self, msg):
        self.aviso = self._json(msg)

    def cb_planeador(self, msg):
        self.planeador = (msg.data, time.time())

    def cb_grelha(self, msg):
        g = np.asarray(msg.data, dtype=np.int8).reshape(int(msg.info.height), int(msg.info.width))
        self.grelha = (g, time.time())

    # --------------------------------------------------------------- envio
    def comando(self, texto):
        msg = String()
        msg.data = texto
        self.pub_comando.publish(msg)

    def imagem_da(self, cam, stamp_ns):
        """A imagem do instante pedido (e esquece as mais antigas), ou None se já não estiver guardada."""
        with self.lock:
            buf = self.imagens[cam]
            dados = buf.get(stamp_ns)
            if dados is not None:
                for t in [t for t in buf if t <= stamp_ns]:
                    del buf[t]
            return dados

    def ultima_imagem(self, cam):
        with self.lock:
            buf = self.imagens[cam]
            if not buf:
                return None, None
            t = next(reversed(buf))
            dados = buf.pop(t)
            buf.clear()
            return t, dados

    def publicar_anotada(self, cam, img, stamp_ns, frame_id):
        if self.pubs_anotada[cam].get_subscription_count() == 0:
            return
        m = ROSImage()
        m.header.stamp.sec, m.header.stamp.nanosec = int(stamp_ns // 1_000_000_000), int(stamp_ns % 1_000_000_000)
        m.header.frame_id = frame_id
        m.height, m.width = img.shape[:2]
        m.encoding, m.is_bigendian, m.step = "bgr8", 0, img.shape[1] * 3
        m.data = array.array("B", img.tobytes())
        self.pubs_anotada[cam].publish(m)


def _terminar(*_):
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, _terminar)
    camaras = visual.ordenar_camaras(dict.fromkeys(os.environ.get("CAMARAS_ATIVAS", "5").split()))
    sem_correcao = os.environ.get("SEM_CORRECAO", "0") == "1"
    direita = camaras[-1] if len(camaras) > 1 else None
    montagem = geometria.carregar_montagem()
    grupo = (montagem or {}).get("grupo", "g05")

    rclpy.init()
    no = Visualizador(camaras, grupo)
    threading.Thread(target=rclpy.spin, args=(no,), daemon=True).start()

    # Calibração de cada câmara (a mesma do processamento): corrigir a imagem antes de desenhar
    calib, corretor = {}, {}
    for c in camaras:
        f = Path(f"config/camera_astra_{c}.yaml")
        if f.exists():
            calib[c] = leitura_bag.carregar_calibracao_yaml(f)
            if not sem_correcao:
                corretor[c] = leitura_bag.CorretorDistorcao(calib[c])
    grelha = grelha_local.GrelhaLocal(montagem) if montagem else None
    p_aviso = None
    if montagem:
        import aviso_faixa
        p_aviso = aviso_faixa.parametros(montagem)

    cv2.namedWindow(JANELA, (cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO) if direita else cv2.WINDOW_AUTOSIZE)
    ecra = {c: None for c in camaras}             # última imagem desenhada de cada câmara
    linhas_chao, geos = {}, {}                    # linhas projetadas no chão e posição das câmaras
    tamanho = (640, 480)
    janela_ajustada = False
    mostrada = False                              # a janela já apareceu (antes disso não pode estar fechada)
    novo = True
    print("Visualizador: R trecho  ·  D câmara da direita  ·  Espaço pausa (reprodução)  ·  Q / Esc terminar")

    def info_painel(cam, p, n_obj, valores, corrigida, sem_det=False):
        modo = p.get("modo", "tempo real")
        rep = no.reproducao or {}
        partes = [f"Câmara {cam}", modo]
        if modo == "reprodução" and rep.get("trecho"):
            partes.append(f"trecho {rep['trecho']}")
        partes.append(p.get("estado_cal") or ("corrigida" if corrigida else "original"))
        tempos = p.get("tempos_ms") or {}
        processamento = p.get("processamento", True)
        nomes = p.get("nome_modelo") or painel.NOME_MODELO
        ativas = [c for c in camaras if c not in (no.estado.get("desligadas") or [])]
        registo_txt = p.get("registo")
        if rep.get("estado") == "pausa":
            registo_txt = "PAUSA  ·  Espaço para retomar"
        freq = p.get("freq")
        return {"tempo": p.get("tempo_s", 0.0), "total": p.get("total_s", 0.0),
                "subtitulo": "  ·  ".join(partes),
                "cor_subtitulo": "#9CA3AF" if corrigida and not sem_det else "#F59E0B",
                "modelos": [(k, nomes.get(k, k), valores.get(k, ""), tempos.get(k), processamento, processamento)
                            for k in painel.MODELOS],
                "rodape": (f"{freq:.1f} deteções/s nesta câmara" if freq else
                           "só gravação (sem processamento)" if not processamento else "à espera das deteções"),
                "registo": registo_txt if ativas and cam == ativas[-1] else None}

    try:
        while rclpy.ok():
            desligadas = set(no.estado.get("desligadas") or [])
            for cam in camaras:
                p = no.painel.get(cam, {})
                if cam in desligadas:
                    if ecra[cam] is not None:
                        ecra[cam] = None
                        linhas_chao.pop(cam, None)
                        novo = True
                    continue
                if p.get("processamento", True) is False:
                    # sem processamento: a imagem mais recente, só com o painel
                    t, dados_img = no.ultima_imagem(cam)
                    if dados_img is None:
                        continue
                    msg = deserialize_message(dados_img, ROSImage)
                    frame = leitura_bag.imagem_bgr(msg)
                    if frame is None:
                        continue
                    ecra[cam] = visual.compor(frame, [], info_painel(cam, p, 0, {}, False, sem_det=True))
                    tamanho = frame.shape[1::-1]
                    novo = True
                    continue
                # a deteção mais recente que ainda tenha a sua imagem guardada
                pendentes = no.dados[cam]
                while pendentes:
                    dados = pendentes.popleft()
                    if pendentes:              # há uma mais recente: esta já não se mostra
                        continue
                    dados_img = no.imagem_da(cam, dados.get("stamp_ns"))
                    if dados_img is None:
                        break
                    msg = deserialize_message(dados_img, ROSImage)
                    frame = leitura_bag.imagem_bgr(msg)
                    if frame is None:
                        break
                    corrigida = bool(dados.get("corrigida")) and cam in corretor
                    if corrigida:
                        frame = corretor[cam](frame)
                    h, w = frame.shape[:2]
                    tamanho = (w, h)
                    caixas, linhas, mascara = painel.deteccoes_de_json(dados, w, h)
                    valores = painel.valores(caixas, linhas, dados.get("estrada_percentagem", 0.0))
                    ecra[cam] = painel.camara(frame, caixas, linhas, mascara,
                                              info_painel(cam, p, len(caixas), valores, corrigida))
                    no.publicar_anotada(cam, ecra[cam], dados["stamp_ns"], msg.header.frame_id)
                    if grelha is not None and cam in montagem["camaras"] and cam in calib:
                        k = geometria.matriz_intrinseca(calib[cam], w, h, corrigida)
                        if k is not None:
                            linhas_chao[cam] = grelha.linhas_no_chao(cam, k, linhas)
                            geos[cam] = grelha.geometria(cam, k)[0]
                    novo = True

            juntas = None
            if novo:
                novo = False
                w, h = tamanho
                espera = {c: (f"Câmara {c}: deteção desligada",
                              "tecla D para voltar a ligar" + (
                                  "" if no.estado.get("modo") == "reprodução" else
                                  " (enquanto desligada, o vídeo desta câmara não é gravado)"))
                          for c in desligadas if c in camaras}
                if all(ecra[c] is None for c in camaras):
                    texto = ("À espera das imagens" if not no.estado else "À espera da primeira imagem")
                    partes = {c: visual.imagem_espera(w, h, f"Câmara {c}: {texto.lower()}",
                                                      "o processamento está a arrancar (modelos)"
                                                      if not no.estado else None) for c in camaras}
                    juntas = painel.juntar(camaras, {**partes, **{c: None for c in espera}}, espera)
                else:
                    juntas = painel.juntar(camaras, ecra, espera)
                if juntas is not None and grelha is not None and no.grelha is not None:
                    g, _ = no.grelha
                    if g.shape == (grelha.altura, grelha.largura):
                        est = no.planeador
                        titulo = ("A*: " + est[0] if est and time.time() - est[1] < 2.0
                                  else "A*: sem estado (planeador?)")
                        avisos = {k: v for k, v in (no.aviso or {}).items() if k in ("esquerda", "direita")}
                        vista = painel.vista_de_cima(grelha, g, juntas.shape[0],
                                                     {c: v for c, v in linhas_chao.items() if c not in desligadas},
                                                     {c: v for c, v in geos.items() if c not in desligadas},
                                                     avisos, p_aviso, titulo)
                        juntas = visual.lado_a_lado([juntas, vista])
                if juntas is not None:
                    if direita and not janela_ajustada and any(v is not None for v in ecra.values()):
                        cv2.resizeWindow(JANELA, juntas.shape[1], juntas.shape[0])
                        janela_ajustada = True
                    cv2.imshow(JANELA, juntas)
            tecla = cv2.waitKey(15) & 0xFF
            fechada = False
            if mostrada:                          # janela fechada no X: termina como a tecla Q
                try:
                    fechada = cv2.getWindowProperty(JANELA, cv2.WND_PROP_VISIBLE) < 1
                except cv2.error:
                    fechada = True
            elif juntas is not None:
                mostrada = True
            if tecla in (ord("q"), 27) or fechada:
                no.comando("terminar")
                time.sleep(0.3)
                break
            if tecla == ord("r"):
                no.comando("trecho")
            elif tecla == ord("d") and direita:
                no.comando("alternar_direita")
            elif tecla == ord(" "):
                no.comando("pausa")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
