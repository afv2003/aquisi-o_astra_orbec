# -*- coding: utf-8 -*-
"""
processar_tempo_real.py - Nó de processamento do online, dentro do Docker (processar_inter.sh).

Só processa e publica: não tem janela. O que se vê no ecrã é feito por outro nó, o visualizador.py
(como recomenda o guião da avaliação: um nó processa e publica, outro subscreve e mostra), e pode
também ver-se no RViz (processar_inter.sh --rviz).

Recebe as imagens das câmaras (/camara5/image_raw, /camara8/image_raw), corrige a distorção da lente
(config/camera_astra_<N>.yaml, como no processar_bag.py), corre os três modelos ONNX (objetos.py,
linhas.py, estrada.py) e publica, por câmara:
  /camara<N>/processamento/dados_deteccoes   std_msgs/String com o JSON das deteções (o mesmo formato
                                             do registo offline, registo.dados_deteccoes, com os
                                             contornos da estrada)
e, para as duas câmaras:
  /processamento/estado_camaras   que câmaras estão a ser processadas (tecla D)
  /processamento/painel           JSON com o que o painel mostra (tempos dos modelos, deteções/s,
                                  correção, indicador de gravação), a cada imagem processada

Decisão local (aula 10): com o config/montagem_camaras.yaml (posição das câmaras no carro), publica
também a posição das câmaras em /tf_static (base_link -> camara<N>_link -> camara<N>) e a grelha
de ocupação local em /<grupo>/grelha_local (nav_msgs/OccupancyGrid no base_link, grelha_local.py),
que o planeador A* comum (src/va_planeamento, serviço "planeador" do compose) usa. A partir da
mesma grelha publica o aviso de mudança de faixa em /<grupo>/aviso_faixa (aviso_faixa.py: para cada
lado, LIVRE / OCUPADA / SEM_FAIXA / SEM_INFORMACAO).

Três modos (o processar_inter.sh escolhe):
* Ao vivo (câmaras): a deteção corre sempre, mas o vídeo original e as deteções só são gravados
  enquanto um trecho estiver ativo (gravacao_online.py). Os trechos começam / terminam com o comando
  "trecho" em /processamento/comando (tecla R no visualizador, ou bash trecho.sh); com
  REGISTAR_TUDO=1 começa logo no arranque. As imagens são gravadas comprimidas (JPEG, o
  /camara<N>/image_raw/compressed do driver). No arranque verifica-se o relógio do PC (relogio.py).
* Reprodução (REPRODUCAO=<bag>): as imagens vêm de um bag (reprodutor.py) nos mesmos tópicos das
  câmaras; o processamento é o mesmo, nada é gravado e os tempos são os do bag.
* Sem processamento (SEM_PROCESSAMENTO=1): só grava os trechos (as câmaras e a gravação, sem os
  modelos), para PCs que não aguentam o processamento.

Comandos em /processamento/comando (std_msgs/String): "trecho", "alternar_direita" (desliga / volta a
ligar a deteção da câmara da direita; enquanto desligada, o vídeo dela não é gravado) e "terminar".

No fim, grava data_realtime_inter/<sessão>/resumo_online.json (trechos gravados, estatísticas das
deteções da sessão inteira, de todos os trechos e de cada trecho, relógio), que o processar_inter.sh
mostra no resumo_online.py.

Variáveis de ambiente (o processar_inter.sh define-as):
  CAMARAS_ATIVAS  "5 8", "5" ou "8"           SEM_CORRECAO   1 = não corrigir a distorção
  SESSAO          nome da pasta da gravação    REGISTAR_TUDO  1 = começar um trecho logo no arranque
  REPRODUCAO      bag a reproduzir (ou vazio)  SEM_PROCESSAMENTO  1 = só gravar
  SEM_INTERFACE   1 = sem visualizador (só para o resumo)   RELOGIO_NTP  yes/no (timedatectl)
"""
import array
import datetime
import json
import os
import signal
import sys
import threading
import time
from collections import deque
from functools import partial
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage, Image as ROSImage
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
import visual
import leitura_bag   # só para a correção da distorção (a mesma do offline)
import registo       # formato das deteções e resumo (os mesmos do offline)
import gravacao_online   # gravação do vídeo original (comprimido) e das deteções, só nos trechos
import geometria         # posição das câmaras no carro (TF) e passagem imagem <-> chão
import grelha_local      # grelha de ocupação local (contrato comum da aula 10)
import aviso_faixa       # aviso de mudança de faixa (zonas da grelha ao lado do carro)
import relogio           # verificação da hora do PC (NTP)

PASTA_REGISTO = Path(os.environ.get("PASTA_REGISTO", "data_realtime_inter"))
TOPICO_ESTADO = "/processamento/estado_camaras"
TOPICO_COMANDO = "/processamento/comando"
TOPICO_PAINEL = "/processamento/painel"
TOPICO_REPRODUCAO = "/processamento/reproducao"
MODELOS = ("objetos", "linhas", "estrada")
QOS_ESTADO = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ProcessadorMultiCamara(Node):
    def __init__(self, camaras, gravar):
        super().__init__('processador_realtime_multi')

        self.ultimo_frame = {cam: None for cam in camaras}   # bytes da última imagem de cada câmara
        self.ultimo_stamp_ns = None                          # instante da última imagem recebida
        self.gravador = None                                 # GravadorTrechos (definido no main)
        self.pubs_dados = {}
        self.comandos = deque()
        self.reproducao = None                               # estado do reprodutor.py (modo reprodução)

        for cam in camaras:
            # raw=True: a imagem chega serializada (bytes); só a que vai ser processada é descodificada
            self.create_subscription(ROSImage, f'/camara{cam}/image_raw',
                                     partial(self.imagem_callback, cam_id=cam), qos_profile_sensor_data, raw=True)
            if gravar:
                # a versão comprimida do driver (JPEG) é a que se grava, tal como chega
                self.create_subscription(CompressedImage, gravacao_online.topico_imagem(cam),
                                         partial(self.comprimida_callback, cam_id=cam), qos_profile_sensor_data,
                                         raw=True)
            self.pubs_dados[cam] = self.create_publisher(String, registo.topico_deteccoes(cam), 10)

        # Estado das câmaras: "transient local", para quem subscrever depois receber o último estado
        # (no bag, o estado é gravado pelo gravacao_online.py, com o instante de cada mudança)
        self.pub_estado = self.create_publisher(String, TOPICO_ESTADO, QOS_ESTADO)
        self.pub_painel = self.create_publisher(String, TOPICO_PAINEL, 10)
        self.create_subscription(String, TOPICO_COMANDO, self.comando_callback, 10)
        self.create_subscription(String, TOPICO_REPRODUCAO, self.reproducao_callback, QOS_ESTADO)
        self.pub_grelha = None
        self.tf_estatico = None
        self.estado_planeador = None
        self.contagem_planeador = {}

    def preparar_tf(self):
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        self.tf_estatico = StaticTransformBroadcaster(self)

    def preparar_grelha(self, topico, grupo):
        """Publicadores da grelha local (nav_msgs/OccupancyGrid) e do aviso de faixa; e subscrição (em
        bruto, para gravar) das saídas do planeador A* comum."""
        from nav_msgs.msg import OccupancyGrid, Path as RosPath
        self.OccupancyGrid = OccupancyGrid
        self.pub_grelha = self.create_publisher(OccupancyGrid, topico, 10)
        self.pub_aviso = self.create_publisher(String, f"/{grupo}/aviso_faixa", 10)
        for chave, tipo, nome in (("estado", String, "estado"), ("caminho", RosPath, "caminho_local"),
                                  ("zona", OccupancyGrid, "zona_proibida")):
            self.create_subscription(tipo, f"/{grupo}/{nome}", partial(self.planeador_callback, chave=chave),
                                     10, raw=True)

    def planeador_callback(self, dados, chave):
        """Saídas do planeador: gravadas tal como chegam (nos trechos); o estado é contado."""
        if chave == "estado":
            texto = deserialize_message(dados, String).data
            self.estado_planeador = (texto, time.time())
            base = texto.split()[0] if texto else "?"
            self.contagem_planeador[base] = self.contagem_planeador.get(base, 0) + 1
        if self.gravador is not None:
            self.gravador.mensagem(f"planeador_{chave}", dados, time.time_ns())

    def publicar_texto(self, pub, texto):
        msg = String()
        msg.data = texto
        pub.publish(msg)
        return texto

    def publicar_aviso(self, texto_json):
        self.publicar_texto(self.pub_aviso, texto_json)

    def publicar_tf_estatico(self, transformadas):
        """transformadas: [(pai, filho, (x, y, z), (qx, qy, qz, qw))]. Devolve a mensagem TFMessage."""
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        agora = self.get_clock().now().to_msg()
        lista = []
        for pai, filho, t, q in transformadas:
            m = TransformStamped()
            m.header.stamp = agora
            m.header.frame_id = pai
            m.child_frame_id = filho
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = t
            (m.transform.rotation.x, m.transform.rotation.y,
             m.transform.rotation.z, m.transform.rotation.w) = q
            lista.append(m)
        self.tf_estatico.sendTransform(lista)
        msg = TFMessage()
        msg.transforms = lista
        return msg

    def publicar_grelha(self, grelha, g, frame, stamp):
        """Grelha (int8, linhas = y) como OccupancyGrid; header.stamp = o da imagem que lhe deu origem."""
        msg = self.OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.info.map_load_time = stamp
        msg.info.resolution = float(g.res)
        msg.info.width = int(g.largura)
        msg.info.height = int(g.altura)
        msg.info.origin.position.x = float(g.x_min)
        msg.info.origin.position.y = float(g.y_min)
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array("b", np.ascontiguousarray(grelha, dtype=np.int8).tobytes())
        self.pub_grelha.publish(msg)
        return msg

    def imagem_callback(self, dados, cam_id):
        self.ultimo_frame[cam_id] = dados
        self.ultimo_stamp_ns = gravacao_online.stamp_da_imagem(dados)
        if self.gravador is not None:
            self.gravador.imagem_bruta(cam_id, dados)   # só se o driver não publicar a comprimida

    def comprimida_callback(self, dados, cam_id):
        if self.gravador is not None:
            self.gravador.imagem(cam_id, dados)         # só grava dentro de um trecho

    def comando_callback(self, msg):
        self.comandos.append(msg.data.strip().lower())

    def reproducao_callback(self, msg):
        try:
            self.reproducao = json.loads(msg.data)
        except ValueError:
            pass

    def publicar_dados(self, cam_id, dados):
        return self.publicar_texto(self.pubs_dados[cam_id], json.dumps(dados, separators=(",", ":")))

    def publicar_estado(self, estado):
        return self.publicar_texto(self.pub_estado, json.dumps(estado, separators=(",", ":")))

    def publicar_painel(self, info):
        self.publicar_texto(self.pub_painel, json.dumps(info, separators=(",", ":"), ensure_ascii=False))


def stamp_ns(ros_msg):
    """Instante da imagem (header.stamp, posto pelo driver da câmara), em ns. É o que liga cada
    deteção à imagem gravada no bag."""
    s = ros_msg.header.stamp
    return int(s.sec) * 1_000_000_000 + int(s.nanosec)


def tempo_txt(seg):
    m, s = divmod(max(0.0, seg), 60)
    return f"{int(m):02d}:{s:04.1f}"


def _terminar(*_):
    """docker stop / docker compose (SIGTERM) termina como o Ctrl+C: o resumo é sempre gravado."""
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, _terminar)

    # Lê as câmaras que o utilizador escolheu no terminal (ex: "5 8", "5", "8"), pela ordem do ecrã
    camaras = visual.ordenar_camaras(dict.fromkeys(os.environ.get('CAMARAS_ATIVAS', '5').split()))
    sem_interface = os.environ.get('SEM_INTERFACE', '0') == '1'
    sem_correcao = os.environ.get('SEM_CORRECAO', '0') == '1'
    sessao = os.environ.get('SESSAO') or f"realtime_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    registar_tudo = os.environ.get('REGISTAR_TUDO', '0') == '1'
    bag_reproducao = os.environ.get('REPRODUCAO', '').strip() or None
    processar = os.environ.get('SEM_PROCESSAMENTO', '0') != '1'
    gravar = bag_reproducao is None
    modo = "reprodução" if bag_reproducao else ("tempo real" if processar else "sem processamento")
    direita = camaras[-1] if len(camaras) > 1 else None      # a câmara que se pode desligar (tecla D)

    # Relógio do PC (só ao vivo): medido em paralelo com o carregamento dos modelos
    est_relogio = {}
    fio_relogio = None
    if gravar:
        fio_relogio = threading.Thread(target=lambda: est_relogio.update(relogio.verificar()), daemon=True)
        fio_relogio.start()

    rclpy.init()
    no_ros = ProcessadorMultiCamara(camaras, gravar)
    thread_ros = threading.Thread(target=rclpy.spin, args=(no_ros,), daemon=True)
    thread_ros.start()

    # --- Modelos (os mesmos do offline, em ONNX) ---
    nome_modelo = {"objetos": "YOLOv8n", "linhas": "CLRNet", "estrada": "YOLOP"}
    detetor_objetos = detetor_linhas = detetor_estrada = None
    if processar:
        import objetos
        import linhas
        import estrada
        detetor_objetos = objetos.DetetorObjetos(confianca=0.25, classes=objetos.CLASSES_ESTRADA)
        detetor_linhas = linhas.DetetorLinhas()
        detetor_estrada = estrada.DetetorEstrada()
        print(f"Objetos (YOLOv8n): {detetor_objetos.dispositivo}")
        print(f"Linhas  (CLRNet):  {detetor_linhas.dispositivo}")
        print(f"Estrada (YOLOP):   {detetor_estrada.dispositivo}")
    else:
        print("Sem processamento (--sem-processamento): só as câmaras e a gravação dos trechos.")
    dispositivos = {k: d.dispositivo for k, d in (("objetos", detetor_objetos), ("linhas", detetor_linhas),
                                                  ("estrada", detetor_estrada)) if d is not None}

    # --- Correção da distorção (config/camera_astra_<N>.yaml de cada câmara, como no offline) ---
    corretor, origem_calib = {}, {}
    # sem correção, a grelha usa a K da calibração (se existir), aproximada
    calibracao_sem_correcao = {c: leitura_bag.carregar_calibracao_yaml(f"config/camera_astra_{c}.yaml")
                               for c in camaras if Path(f"config/camera_astra_{c}.yaml").exists()}
    if not processar:
        pass
    elif sem_correcao:
        print("Correção da distorção: DESLIGADA (--sem-correcao)")
    else:
        for cam in camaras:
            ficheiro = Path(f"config/camera_astra_{cam}.yaml")
            if ficheiro.exists():
                corretor[cam] = leitura_bag.CorretorDistorcao(leitura_bag.carregar_calibracao_yaml(ficheiro))
                origem_calib[cam] = str(ficheiro)
                print(f"Câmara {cam}: correção da distorção com {ficheiro}")
            else:
                print(f"Aviso: câmara {cam}: '{ficheiro}' não existe; a imagem NÃO é corrigida (README, secção 2).")

    # --- Decisão local: posição das câmaras no carro (TF) e grelha de ocupação local ---
    montagem = geometria.carregar_montagem()
    grelha = topico_grelha = None
    if montagem is None:
        print(f"Aviso: falta {geometria.CONFIG}; sem TF das câmaras nem grelha local.")
    else:
        try:
            no_ros.preparar_tf()
        except ImportError as e:
            print(f"Aviso: sem TF das câmaras ({e}).")
        if processar:
            sem_montagem = [c for c in camaras if c not in montagem["camaras"]]
            topico_grelha = f"/{montagem.get('grupo', 'g05')}/grelha_local"
            try:
                no_ros.preparar_grelha(topico_grelha, montagem.get("grupo", "g05"))
                grelha = grelha_local.GrelhaLocal(montagem)
                print(f"Grelha local: {topico_grelha} (nav_msgs/OccupancyGrid, "
                      f"{montagem.get('frame_carro', 'base_link')}, {grelha.res:g} m, x {grelha.x_min:g} a "
                      f"{grelha.x_max:g} m, y {grelha.y_min:g} a {grelha.y_max:g} m)"
                      + (f"; sem posição no yaml: câmara {', '.join(sem_montagem)}" if sem_montagem else ""))
            except ImportError as e:
                print(f"Aviso: sem grelha local ({e}).")

    # --- Relógio do PC (o resultado fica no bag e no resumo) ---
    if fio_relogio is not None:
        fio_relogio.join(timeout=5)
        print(relogio.texto_completo(est_relogio) if est_relogio else "Relógio do PC: não foi possível verificar.")

    # --- Gravação por trechos (o bag começa agora: os tempos do bag são os da sessão) ---
    extra = {}
    if grelha is not None:
        grupo = montagem.get("grupo", "g05")
        extra = {"grelha": (topico_grelha, "nav_msgs/msg/OccupancyGrid", gravacao_online.DEFINICAO_GRELHA),
                 "aviso": (f"/{grupo}/aviso_faixa", "std_msgs/msg/String", gravacao_online.DEFINICAO_STRING),
                 "planeador_estado": (f"/{grupo}/estado", "std_msgs/msg/String", gravacao_online.DEFINICAO_STRING),
                 "planeador_caminho": (f"/{grupo}/caminho_local", "nav_msgs/msg/Path",
                                       gravacao_online.DEFINICAO_CAMINHO),
                 "planeador_zona": (f"/{grupo}/zona_proibida", "nav_msgs/msg/OccupancyGrid",
                                    gravacao_online.DEFINICAO_GRELHA)}
    if no_ros.tf_estatico is not None:
        extra["tf_static"] = ("/tf_static", "tf2_msgs/msg/TFMessage", gravacao_online.DEFINICAO_TF)
    if gravar:
        gravador = gravacao_online.GravadorTrechos(PASTA_REGISTO / sessao / f"{sessao}_0.mcap", camaras, extra,
                                                   relogio=est_relogio or None)
    else:
        gravador = gravacao_online.GravadorNulo(camaras)
    if no_ros.tf_estatico is not None:
        frame_carro = montagem.get("frame_carro", "base_link")
        transformadas = []
        for c in camaras:
            if c in montagem["camaras"]:
                geo = geometria.GeometriaCamara(montagem["camaras"][c], np.eye(3))
                t, q = geo.tf_corpo()
                transformadas.append((frame_carro, f"camara{c}_link", t, q))
                t, q = geo.tf_otico()
                transformadas.append((f"camara{c}_link", f"camara{c}", t, q))
        msg_tf = no_ros.publicar_tf_estatico(transformadas)
        gravador.mensagem("tf_static", serialize_message(msg_tf), gravador.inicio_ns, sempre=True)
        print("TF estático: " + ", ".join(f"{p} -> {f}" for p, f, _, _ in transformadas))
    no_ros.gravador = gravador

    # --- Tempo: ao vivo, o relógio do PC desde o início da sessão; em reprodução, o tempo do bag ---
    def agora_ns():
        if gravar:
            return time.time_ns()
        return no_ros.ultimo_stamp_ns or inicio_ns()

    def inicio_ns():
        if gravar:
            return gravador.inicio_ns
        rep = no_ros.reproducao or {}
        return rep.get("inicio_ns") or no_ros.ultimo_stamp_ns or gravador.inicio_ns

    def agora_s():
        return (agora_ns() - inicio_ns()) / 1e9

    # --- Estado ---
    inicio = datetime.datetime.now()
    ativa = {c: True for c in camaras}
    desligada_desde = {c: None for c in camaras}
    desligada = {c: [] for c in camaras}          # [(início_s, fim_s)] desde o arranque
    tempos = {k: None for k in MODELOS + ("grelha",)}   # média móvel (ms) de cada modelo, para o painel
    soma_ms = {k: 0.0 for k in MODELOS + ("grelha", "total")}
    n_img = {c: 0 for c in camaras}
    freq = {c: None for c in camaras}             # deteções por segundo de cada câmara (média móvel)
    t_ant = {c: None for c in camaras}
    resumo_det = {c: {} for c in camaras}         # deteções em memória (compactas), para o resumo do fim
    p_aviso = aviso_faixa.parametros(montagem)
    ultimo_aviso = {}                             # último texto mostrado no terminal, por lado
    contagem_aviso = {l: {} for l in aviso_faixa.LADOS}          # estados do aviso (sessão inteira)
    contagem_aviso_trechos = {l: {} for l in aviso_faixa.LADOS}  # estados do aviso (só nos trechos)
    t_painel = {c: 0.0 for c in camaras}          # sem processamento: último painel publicado

    def media(k, ms):
        tempos[k] = ms if tempos[k] is None else 0.9 * tempos[k] + 0.1 * ms
        soma_ms[k] += ms

    def texto_registo():
        if gravador.trecho is None:
            return None
        dur = time.time() - gravador.trecho["pedido_ns"] / 1e9
        return f"REGISTO  ·  trecho {gravador.trecho['n']}  ·  {tempo_txt(dur)}"

    def publicar_estado():
        ativas = [c for c in camaras if ativa[c]]
        texto = no_ros.publicar_estado({"stamp_ns": time.time_ns(), "ativas": ativas,
                                        "desligadas": [c for c in camaras if not ativa[c]],
                                        "modo": modo, "pronto": True})
        gravador.estado(ativas, texto)

    def publicar_painel(cam, corrigida, extra_info=None):
        info = {"camara": cam, "modo": modo, "tempo_s": round(agora_s(), 2), "corrigida": corrigida,
                "estado_cal": ("corrigida" if corrigida else "sem correção" if sem_correcao
                               else "sem calibração"),
                "tempos_ms": {k: (round(v, 1) if v is not None else None) for k, v in tempos.items()},
                "freq": round(freq[cam], 2) if freq[cam] else None, "registo": texto_registo(),
                "nome_modelo": nome_modelo, "dispositivos": dispositivos, "processamento": processar}
        rep = no_ros.reproducao or {}
        if rep.get("duracao_s"):
            info["total_s"] = rep["duracao_s"]
        if extra_info:
            info.update(extra_info)
        no_ros.publicar_painel(info)

    def alternar_trecho():
        """Tecla R / trecho.sh: começa um trecho (grava o vídeo original e as deteções) ou termina-o."""
        if not gravar:
            print(f"[{tempo_txt(agora_s())}] Em reprodução nada é gravado (os dados já estão no bag).")
            return
        if gravador.trecho is None:
            n = gravador.iniciar_trecho()
            print(f"[{tempo_txt(agora_s())}] Trecho {n} iniciado: a gravar o vídeo original"
                  + (" e as deteções" if processar else "")
                  + ("" if all(ativa.values()) else
                     f" (só da câmara {', '.join(c for c in camaras if ativa[c])}: a outra está desligada)"))
        else:
            t = gravador.terminar_trecho()
            print(f"[{tempo_txt(agora_s())}] Trecho {t['n']} terminado: {tempo_txt(t['inicio_s'])} a "
                  f"{tempo_txt(t['fim_s'])}, " + ", ".join(f"câmara {c}: {k} imagens"
                                                         for c, k in t["imagens_por_camara"].items()))

    def alternar(cam):
        """Liga / desliga a deteção da câmara da direita (a da esquerda fica sempre ligada)."""
        if cam != direita:
            return
        agora = agora_s()
        ativa[cam] = not ativa[cam]
        if grelha is not None:
            grelha.remover(cam)                   # a parte desta câmara fica desconhecida
        if ativa[cam]:
            desligada[cam].append((desligada_desde[cam], agora))
            desligada_desde[cam] = None
            no_ros.ultimo_frame[cam] = None        # recomeça na próxima imagem (a mais recente)
            t_ant[cam] = None
            print(f"[{tempo_txt(agora)}] Câmara {cam}: deteção LIGADA")
        else:
            desligada_desde[cam] = agora
            print(f"[{tempo_txt(agora)}] Câmara {cam}: deteção DESLIGADA "
                  f"(a câmara {', '.join(c for c in camaras if ativa[c])} passa a ser processada mais vezes"
                  + ("; o vídeo desta câmara deixa de ser gravado nos trechos)" if gravar else ")"))
        publicar_estado()

    publicar_estado()                              # também diz ao reprodutor que já se pode começar
    if gravar:
        print(f"Sessão: {sessao}  (gravação só nos trechos, imagens em JPEG: {gravador.caminho})")
    else:
        print(f"Reprodução de: {bag_reproducao}  (nada é gravado; os tempos são os do bag)")
    print(f"A aguardar imagens da(s) câmara(s) {', '.join(camaras)} ({modo}; "
          + ("sem visualizador)" if sem_interface else "o ecrã é o do visualizador)") + "...")
    print("--- CONTROLOS (na janela do visualizador, ou com os scripts noutro terminal) ---")
    if gravar:
        print("[R]      : Começar / terminar um trecho (só nos trechos se grava o vídeo original e as deteções)"
              "  - ou: bash trecho.sh")
    else:
        print("[Espaço] : Pausar / retomar a reprodução")
    if direita and processar:
        print(f"[D]      : Desligar / voltar a ligar a deteção da câmara {direita} (direita)"
              "  - ou: bash alternar_camara.sh")
    print("[Q] [Esc]: Terminar  (ou Ctrl + C neste terminal)")
    if registar_tudo and gravar:
        alternar_trecho()
    elif gravar:
        print(("A deteção já está a correr; " if processar else "") + "nada é gravado até começar um trecho.")

    try:
        while rclpy.ok():
            # comandos recebidos pelo tópico (visualizador, trecho.sh, alternar_camara.sh)
            sair = False
            while no_ros.comandos:
                cmd = no_ros.comandos.popleft()
                if cmd in ("alternar", "alternar_direita") and direita and processar:
                    alternar(direita)
                elif cmd == "trecho":
                    alternar_trecho()
                elif cmd == "terminar":
                    sair = True
            if sair:
                print(f"[{tempo_txt(agora_s())}] Terminado pelo visualizador (tecla Q).")
                break
            rep = no_ros.reproducao or {}
            if not gravar and rep.get("estado") == "fim" and all(no_ros.ultimo_frame[c] is None
                                                                for c in camaras if ativa[c]):
                print(f"[{tempo_txt(agora_s())}] Fim do bag.")
                break

            if not processar:
                # só gravação: o painel do visualizador diz o estado da gravação, 2 vezes por segundo
                for cam in camaras:
                    if time.time() - t_painel[cam] > 0.5:
                        t_painel[cam] = time.time()
                        publicar_painel(cam, False, {"estado_cal": "imagem original"})
                time.sleep(0.05)
                continue

            processou = False
            for cam in camaras:
                dados_img = no_ros.ultimo_frame[cam]
                if dados_img is None or not ativa[cam]:
                    continue
                no_ros.ultimo_frame[cam] = None
                msg = deserialize_message(dados_img, ROSImage)
                frame = leitura_bag.imagem_bgr(msg)
                if frame is None:
                    continue
                processou = True
                t_inicio = time.perf_counter()
                corrigida = cam in corretor
                if corrigida:
                    frame = corretor[cam](frame)

                # Processamento (todos em ONNX), sobre a imagem corrigida e ainda sem desenhos
                t = time.perf_counter()
                linhas_frame = detetor_linhas.detetar(frame)
                media("linhas", (time.perf_counter() - t) * 1000)
                t = time.perf_counter()
                mascara_estrada = detetor_estrada.detetar(frame)
                media("estrada", (time.perf_counter() - t) * 1000)
                t = time.perf_counter()
                caixas = detetor_objetos.detetar(frame)
                media("objetos", (time.perf_counter() - t) * 1000)

                # Publicar as deteções (o mesmo formato do registo offline, com os contornos da estrada,
                # que o visualizador desenha)
                t_img = stamp_ns(msg)
                dados = registo.dados_deteccoes(caixas, linhas_frame, mascara_estrada, t_img,
                                                corrigida=corrigida, camara=cam)
                texto = no_ros.publicar_dados(cam, dados)
                trecho = gravador.deteccoes(cam, texto, t_img)    # gravada só dentro de um trecho

                # Grelha local: a camada desta câmara (a partir das mesmas deteções) e a grelha junta
                if grelha is not None and cam in montagem["camaras"]:
                    calib = corretor[cam].calibracao if cam in corretor else calibracao_sem_correcao.get(cam)
                    k = geometria.matriz_intrinseca(calib, frame.shape[1], frame.shape[0], corrigida)
                    if k is not None:
                        t = time.perf_counter()
                        grelha.atualizar(cam, k, frame.shape[1], frame.shape[0], t_img, mascara_estrada, caixas)
                        grelha_atual = grelha.combinada(t_img)
                        msg_g = no_ros.publicar_grelha(grelha_atual, grelha, montagem.get("frame_carro", "base_link"),
                                                       msg.header.stamp)
                        if gravador.trecho is not None:
                            gravador.mensagem("grelha", serialize_message(msg_g), t_img)
                        media("grelha", (time.perf_counter() - t) * 1000)

                        # Aviso de mudança de faixa (a partir da mesma grelha)
                        avisos = aviso_faixa.avaliar(grelha, grelha_atual, grelha.veiculos(t_img), p_aviso)
                        texto_aviso = json.dumps({"stamp_ns": t_img, **avisos}, separators=(",", ":"))
                        no_ros.publicar_aviso(texto_aviso)
                        if gravador.trecho is not None:
                            gravador.mensagem("aviso", gravacao_online.cdr_string(texto_aviso), t_img)
                        for lado, info in avisos.items():
                            contagem_aviso[lado][info["estado"]] = contagem_aviso[lado].get(info["estado"], 0) + 1
                            if trecho is not None:
                                c_ = contagem_aviso_trechos[lado]
                                c_[info["estado"]] = c_.get(info["estado"], 0) + 1
                            txt = aviso_faixa.texto(info)
                            if txt != ultimo_aviso.get(lado):
                                ultimo_aviso[lado] = txt
                                print(f"[{tempo_txt(agora_s())}] Faixa da {lado}: {txt}")
                # resumo em memória: só o que as estatísticas usam (e o trecho, para o filtro do resumo)
                resumo_det[cam][t_img] = {"objetos": [{"classe": o["classe"], "confianca": o["confianca"]}
                                                      for o in dados["objetos"]],
                                          "n_linhas": len(dados["linhas"]),
                                          "estrada_percentagem": dados["estrada_percentagem"],
                                          "trecho": trecho}
                n_img[cam] += 1
                agora = time.perf_counter()
                if t_ant[cam] is not None and agora > t_ant[cam]:
                    f = 1 / (agora - t_ant[cam])
                    freq[cam] = f if freq[cam] is None else 0.9 * freq[cam] + 0.1 * f
                t_ant[cam] = agora
                soma_ms["total"] += (time.perf_counter() - t_inicio) * 1000
                publicar_painel(cam, corrigida, {"stamp_ns": t_img})

            if not processou:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        fim = agora_s()
        for c in camaras:
            if desligada_desde[c] is not None:
                desligada[c].append((desligada_desde[c], fim))
        trechos = []
        try:
            trechos = gravador.fechar()
            if gravar:
                mostrar_trechos(trechos, gravador)
        except Exception as e:
            print(f"Aviso: a gravação não terminou bem ({type(e).__name__}: {e}); o bag pode ficar sem "
                  "índice (o offline lê-o na mesma).")
        try:
            gravar_resumo(sessao, inicio, fim, camaras, corretor, origem_calib, sem_correcao, sem_interface,
                          n_img, desligada, soma_ms, detetor_objetos, detetor_linhas, detetor_estrada,
                          nome_modelo, resumo_det, trechos, gravador,
                          avisos={"sessao": contagem_aviso, "trechos": contagem_aviso_trechos,
                                  "planeador": dict(no_ros.contagem_planeador)} if grelha is not None else None,
                          info_grelha=None if grelha is None else {
                              "topico": topico_grelha, "frame": montagem.get("frame_carro", "base_link"),
                              "resolucao": grelha.res, "x": [grelha.x_min, grelha.x_max],
                              "y": [grelha.y_min, grelha.y_max], "camaras": montagem["camaras"]},
                          modo=modo, relogio_pc=est_relogio or None,
                          origem=None if gravar else {"tipo": "reproducao", "bag": bag_reproducao,
                                                      **{k: v for k, v in (no_ros.reproducao or {}).items()
                                                         if k in ("inicio_ns", "duracao_s", "trechos", "relogio")}})
        except Exception as e:
            print(f"Aviso: não foi possível gravar o resumo ({type(e).__name__}: {e}).")
        if rclpy.ok():
            rclpy.shutdown()


def mostrar_trechos(trechos, gravador):
    gravados = [t for t in trechos if t["imagens"]]
    if not gravados:
        print("\nNenhum trecho gravado: a sessão não guardou vídeo (só o resumo das deteções).")
        return
    tamanho = gravador.tamanho_bytes / 1e9
    print(f"\nGravação: {gravador.caminho}  ({tamanho:.2f} GB, {len(gravados)} trecho(s))")
    for t in trechos:
        print(f"  trecho {t['n']}: {tempo_txt(t['inicio_s'])} a {tempo_txt(t['fim_s'])}  ("
              + ", ".join(f"câmara {c}: {k} imagens, {t['deteccoes_por_camara'].get(c, 0)} com deteções"
                          for c, k in t["imagens_por_camara"].items()) + ")")


def _resumos_por_camara(resumo_det, camaras, filtro):
    """{"Ambas": ..., "5": ..., "8": ...} só com as deteções em que filtro(dados) é verdadeiro."""
    sel = {c: {t: d for t, d in resumo_det[c].items() if filtro(d)} for c in camaras}
    resumos = {}
    if len(camaras) > 1:
        resumos["Ambas"] = registo.resumo_deteccoes(sel)
    for c in camaras:
        resumos[c] = registo.resumo_deteccoes({c: sel[c]})
    return resumos


def _trecho_para_json(t, gravador):
    """Trecho como fica no resumo (tempos em s desde o início da sessão)."""
    def rel(x):
        return round((x - gravador.inicio_ns) / 1e9, 3)
    return {"n": t["n"], "inicio_ns": t["inicio_ns"], "fim_ns": t["fim_ns"],
            "inicio_s": t["inicio_s"], "fim_s": t["fim_s"],
            "duracao_s": round(t["fim_s"] - t["inicio_s"], 3),
            "imagens": t["imagens"], "imagens_por_camara": t["imagens_por_camara"],
            "deteccoes_por_camara": t["deteccoes_por_camara"],
            "camaras_gravadas_s": {c: [[rel(a), rel(b)] for a, b in ints]
                                   for c, ints in t["intervalos_por_camara"].items()},
            # partes do trecho em que a câmara esteve desligada (tecla D): sem vídeo nem deteções dela
            "camaras_desligadas_s": {c: [[rel(a), rel(b)] for a, b in _complemento(
                ints, t["evento_inicio_ns"], t["evento_fim_ns"]) if b - a > 50_000_000]
                for c, ints in t["intervalos_por_camara"].items()}}


def _complemento(intervalos, inicio, fim):
    """Partes de [inicio, fim] fora dos intervalos dados (ordenados)."""
    res, cursor = [], inicio
    for a, b in sorted(intervalos):
        if a > cursor:
            res.append((cursor, a))
        cursor = max(cursor, b)
    if cursor < fim:
        res.append((cursor, fim))
    return res


def gravar_resumo(sessao, inicio, duracao_s, camaras, corretor, origem_calib, sem_correcao, sem_interface,
                  n_img, desligada, soma_ms, det_obj, det_lin, det_est, nome_modelo, resumo_det,
                  trechos=(), gravador=None, info_grelha=None, avisos=None, modo="tempo real", relogio_pc=None,
                  origem=None):
    """Estatísticas da sessão (como no Resumo do offline) em data_realtime_inter/<sessão>/resumo_online.json."""
    total = sum(n_img.values())
    # Filtros do resumo: a sessão inteira (tudo o que foi detetado), só os trechos gravados, e cada trecho
    resumos_por_filtro = {"sessao": _resumos_por_camara(resumo_det, camaras, lambda d: True)}
    if trechos:
        resumos_por_filtro["trechos"] = _resumos_por_camara(resumo_det, camaras,
                                                            lambda d: d.get("trecho") is not None)
        for t in trechos:
            resumos_por_filtro[str(t["n"])] = _resumos_por_camara(
                resumo_det, camaras, lambda d, n=t["n"]: d.get("trecho") == n)
    resumos = resumos_por_filtro["sessao"]
    ativa_s = {c: duracao_s - sum(b - a for a, b in desligada[c]) for c in camaras}
    resumo = {
        "tipo": "resumo_online",
        "processado_com": "processar_tempo_real.py (online)",
        "modo": modo,                               # "tempo real", "reprodução" ou "sem processamento"
        "origem": origem,                           # reprodução: o bag reproduzido (senão None: câmaras)
        "relogio": relogio_pc,                      # relógio do PC no arranque (NTP; relogio.py)
        "sessao": sessao,
        "inicio": inicio.isoformat(timespec="seconds"),
        "duracao_s": round(duracao_s, 1),
        "com_interface": not sem_interface,
        "camaras": [{"camara": c,
                     "topico_imagem": f"/camara{c}/image_raw",
                     "topico_gravado": gravacao_online.topico_imagem(c) if origem is None else None,
                     "topico_deteccoes": registo.topico_deteccoes(c),
                     "correcao": origem_calib[c] if c in corretor else
                     ("desligada (--sem-correcao)" if sem_correcao else None),
                     "imagens_processadas": n_img[c],
                     "deteccoes_por_segundo": round(n_img[c] / ativa_s[c], 2) if ativa_s[c] > 0 else None,
                     "desligada_s": [[round(a, 1), round(b, 1)] for a, b in desligada[c]]} for c in camaras],
        "modelos": {} if det_obj is None else {
            "objetos": {"nome": nome_modelo["objetos"], "dispositivo": det_obj.dispositivo,
                        "confianca_minima": det_obj.confianca,
                        "classes": [det_obj.nomes[i] for i in det_obj.classes_ids]
                        if det_obj.classes_ids is not None else "todas"},
            "linhas": {"nome": nome_modelo["linhas"], "dispositivo": det_lin.dispositivo,
                       "confianca_minima": det_lin.cfg["conf_threshold"], "corte_topo": det_lin.cfg["corte_topo"]},
            "estrada": {"nome": nome_modelo["estrada"], "dispositivo": det_est.dispositivo,
                        "confianca_minima": det_est.confianca},
        },
        "tempo_medio_ms": {k: round(soma_ms[k] / total, 1) for k in soma_ms} if total else {},
        # gravação: só os trechos (vídeo original e deteções); tempos em s desde o início da sessão
        "gravacao": {"ficheiro": gravador.caminho.name if gravador and gravador.tamanho_bytes else None,
                     "tamanho_bytes": gravador.tamanho_bytes if gravador else 0,
                     "inicio_ns": gravador.inicio_ns if gravador else None,
                     "imagens": "JPEG (sensor_msgs/CompressedImage)" if origem is None else None,
                     # imagens comprimidas por este programa (o driver não publicou a versão comprimida)
                     "comprimidas_pelo_processamento": dict(gravador.comprimidas_aqui) if gravador else {}},
        "trechos": [_trecho_para_json(t, gravador) for t in trechos],
        "grelha_local": info_grelha,                # tópico, limites e montagem das câmaras (ou None)
        # aviso de mudança de faixa (imagens processadas em cada estado, por lado) e estados do planeador
        "decisao_local": avisos,
        "resumos": resumos,                         # a sessão inteira (como antes)
        "resumos_por_filtro": resumos_por_filtro,   # "sessao", "trechos" e "1", "2", ... (cada trecho)
    }
    pasta = PASTA_REGISTO / sessao
    pasta.mkdir(parents=True, exist_ok=True)
    caminho = pasta / "resumo_online.json"
    caminho.write_text(json.dumps(resumo, indent=2, ensure_ascii=False), encoding="utf-8")
    if relogio_pc is not None and not relogio_pc.get("ok"):
        import relogio
        print(f"Relógio do PC: {relogio.resumo_curto(relogio_pc)}")
    print(f"\nDuração: {tempo_txt(duracao_s)}  ·  imagens processadas: "
          + ", ".join(f"câmara {c}: {n_img[c]}" for c in camaras))
    print(f"Resumo gravado em: {caminho}")


if __name__ == '__main__':
    main()
