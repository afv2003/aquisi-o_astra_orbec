# -*- coding: utf-8 -*-
"""
processar_tempo_real.py - Processamento em tempo real (online), dentro do Docker (processar_inter.sh).

Recebe as imagens das câmaras (/camara5/image_raw, /camara8/image_raw), corrige a distorção da lente
(config/camera_astra_<N>.yaml, como no processar_bag.py), corre os três modelos ONNX (objetos.py,
linhas.py, estrada.py) e publica, por câmara:
  /camara<N>/processamento/dados_deteccoes   std_msgs/String com o JSON das deteções (mesmo formato
                                             do registo offline, registo.dados_deteccoes)
  /camara<N>/processamento/imagem_anotada    sensor_msgs/Image com os desenhos (só com janela, ou se
                                             alguém estiver a subscrever o tópico)
e, para as duas câmaras, /processamento/estado_camaras (que câmaras estão a ser processadas).

Gravação por trechos (gravacao_online.py): a deteção corre sempre, mas o vídeo original e as deteções
só são gravados enquanto um trecho estiver ativo, como no registo do offline:
  [R] na janela, ou bash trecho.sh sem janela (tópico /processamento/comando), começa / termina um
  trecho; com REGISTAR_TUDO=1 (processar_inter.sh --registar-tudo) começa logo no arranque.
O bag fica em data_realtime_inter/<sessão>/<sessão>_0.mcap, só com os trechos; com a deteção de uma
câmara desligada (tecla D), o vídeo dessa câmara também não é gravado.

Com janela: uma só janela com as câmaras lado a lado (5 à esquerda, 8 à direita), como no offline.
  [R] começa / termina um trecho (gravação);
  [D] desliga / volta a ligar a deteção da câmara da direita (a outra passa a ser processada mais
      vezes por segundo; o vídeo da câmara desligada deixa de ser gravado); [Q] ou [Esc] termina.
Sem janela (SEM_INTERFACE=1): o mesmo, sem desenhar; os trechos com o trecho.sh e a câmara da direita
  com o alternar_camara.sh (tópico /processamento/comando).

No fim, grava data_realtime_inter/<sessão>/resumo_online.json (trechos gravados e estatísticas das
deteções da sessão inteira, de todos os trechos e de cada trecho, como no Resumo do offline), que o
processar_inter.sh mostra no resumo_online.py (só com janela).

Variáveis de ambiente (o processar_inter.sh define-as):
  CAMARAS_ATIVAS  "5 8", "5" ou "8"          SEM_INTERFACE  1 = sem janela
  SEM_CORRECAO    1 = não corrigir a distorção SESSAO         nome da pasta da gravação
  REGISTAR_TUDO   1 = começar um trecho logo no arranque (gravar a sessão toda)
"""
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

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
import objetos
import linhas
import estrada
import visual
import leitura_bag   # só para a correção da distorção (a mesma do offline)
import registo       # formato das deteções e resumo (os mesmos do offline)
import gravacao_online   # gravação do vídeo original e das deteções, só nos trechos

PASTA_REGISTO = Path(os.environ.get("PASTA_REGISTO", "data_realtime_inter"))
TOPICO_ESTADO = "/processamento/estado_camaras"
TOPICO_COMANDO = "/processamento/comando"
JANELA = "Tempo Real"
MODELOS = ("objetos", "linhas", "estrada")


class ProcessadorMultiCamara(Node):
    def __init__(self, camaras):
        super().__init__('processador_realtime_multi')

        self.ultimo_frame = {cam: None for cam in camaras}   # bytes da última imagem de cada câmara
        self.gravador = None                                 # GravadorTrechos (definido no main)
        self.pubs_dados = {}
        self.pubs_imagem = {}
        self.comandos = deque()

        for cam in camaras:
            # Subscritor dinâmico para cada câmara
            cb = partial(self.listener_callback, cam_id=cam)
            # raw=True: a imagem chega serializada (bytes); é gravada assim, sem descodificar, e só a
            # que vai ser processada é descodificada
            self.create_subscription(
                ROSImage, f'/camara{cam}/image_raw', cb, qos_profile_sensor_data, raw=True)

            # Publicadores dinâmicos para cada câmara
            self.pubs_dados[cam] = self.create_publisher(
                String, registo.topico_deteccoes(cam), 10)
            self.pubs_imagem[cam] = self.create_publisher(
                ROSImage, f'/camara{cam}/processamento/imagem_anotada', 10)

        # Estado das câmaras: "transient local", para quem subscrever depois receber o último estado
        # (no bag, o estado é gravado pelo gravacao_online.py, com o instante de cada mudança)
        qos_estado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_estado = self.create_publisher(String, TOPICO_ESTADO, qos_estado)
        # Comandos (sem janela: alternar_camara.sh)
        self.create_subscription(String, TOPICO_COMANDO, self.comando_callback, 10)

    def listener_callback(self, dados, cam_id):
        self.ultimo_frame[cam_id] = dados
        if self.gravador is not None:
            self.gravador.imagem(cam_id, dados)      # só grava dentro de um trecho

    def comando_callback(self, msg):
        self.comandos.append(msg.data.strip().lower())

    def publicar_dados(self, cam_id, dados):
        msg_string = String()
        msg_string.data = json.dumps(dados, separators=(",", ":"))
        self.pubs_dados[cam_id].publish(msg_string)
        return msg_string.data

    def quer_imagem_anotada(self, cam_id):
        return self.pubs_imagem[cam_id].get_subscription_count() > 0

    def publicar_imagem(self, cam_id, frame_anotado, ros_msg_original):
        msg_img = ROSImage()
        msg_img.header = ros_msg_original.header
        msg_img.height = frame_anotado.shape[0]
        msg_img.width = frame_anotado.shape[1]
        msg_img.encoding = 'bgr8'
        msg_img.is_bigendian = 0
        msg_img.step = frame_anotado.shape[1] * 3
        msg_img.data = frame_anotado.tobytes()
        self.pubs_imagem[cam_id].publish(msg_img)

    def publicar_estado(self, estado):
        msg = String()
        msg.data = json.dumps(estado, separators=(",", ":"))
        self.pub_estado.publish(msg)
        return msg.data


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
    direita = camaras[-1] if len(camaras) > 1 else None      # a câmara que se pode desligar (tecla D)

    rclpy.init()
    no_ros = ProcessadorMultiCamara(camaras)
    thread_ros = threading.Thread(target=rclpy.spin, args=(no_ros,), daemon=True)
    thread_ros.start()

    # --- Modelos (os mesmos do offline, em ONNX) ---
    detetor_objetos = objetos.DetetorObjetos(confianca=0.25, classes=objetos.CLASSES_ESTRADA)
    detetor_linhas = linhas.DetetorLinhas()
    detetor_estrada = estrada.DetetorEstrada()
    print(f"Objetos (YOLOv8n): {detetor_objetos.dispositivo}")
    print(f"Linhas  (CLRNet):  {detetor_linhas.dispositivo}")
    print(f"Estrada (YOLOP):   {detetor_estrada.dispositivo}")
    nome_modelo = {"objetos": "YOLOv8n", "linhas": "CLRNet", "estrada": "YOLOP"}

    # --- Correção da distorção (config/camera_astra_<N>.yaml de cada câmara, como no offline) ---
    corretor, origem_calib = {}, {}
    if sem_correcao:
        print("Correção da distorção: DESLIGADA (--sem-correcao)")
    else:
        for cam in camaras:
            ficheiro = Path(f"config/camera_astra_{cam}.yaml")
            if ficheiro.exists():
                corretor[cam] = leitura_bag.CorretorDistorcao(leitura_bag.carregar_calibracao_yaml(ficheiro))
                origem_calib[cam] = str(ficheiro)
                print(f"Câmara {cam}: correção da distorção com {ficheiro}")
            else:
                print(f"Aviso: câmara {cam}: '{ficheiro}' não existe; a imagem NÃO é corrigida (README, secção 3).")

    if not sem_interface:
        # uma só janela, com as câmaras lado a lado; com duas, pode ser redimensionada (mantém a proporção)
        cv2.namedWindow(JANELA, (cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO) if direita else cv2.WINDOW_AUTOSIZE)

    # --- Gravação por trechos (o bag começa agora: os tempos do bag são os da sessão) ---
    gravador = gravacao_online.GravadorTrechos(PASTA_REGISTO / sessao / f"{sessao}_0.mcap", camaras)
    no_ros.gravador = gravador

    # --- Estado ---
    t0_wall = gravador.inicio_ns / 1e9
    inicio = datetime.datetime.now()
    ativa = {c: True for c in camaras}
    desligada_desde = {c: None for c in camaras}
    desligada = {c: [] for c in camaras}          # [(início_s, fim_s)] desde o arranque
    ecra = {c: None for c in camaras}             # última imagem desenhada de cada câmara
    tempos = {k: None for k in MODELOS}           # média móvel (ms) de cada modelo, para o painel
    soma_ms = {k: 0.0 for k in MODELOS + ("total",)}
    n_img = {c: 0 for c in camaras}
    freq = {c: None for c in camaras}             # deteções por segundo de cada câmara (média móvel)
    t_ant = {c: None for c in camaras}
    resumo_det = {c: {} for c in camaras}         # deteções em memória (compactas), para o resumo do fim
    janela_ajustada = False

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
                                        "desligadas": [c for c in camaras if not ativa[c]]})
        gravador.estado(ativas, texto)

    def alternar_trecho():
        """Tecla R / trecho.sh: começa um trecho (grava o vídeo original e as deteções) ou termina-o."""
        agora = time.time() - t0_wall
        if gravador.trecho is None:
            n = gravador.iniciar_trecho()
            print(f"[{tempo_txt(agora)}] Trecho {n} iniciado: a gravar o vídeo original e as deteções"
                  + ("" if all(ativa.values()) else
                     f" (só da câmara {', '.join(c for c in camaras if ativa[c])}: a outra está desligada)"))
        else:
            t = gravador.terminar_trecho()
            print(f"[{tempo_txt(agora)}] Trecho {t['n']} terminado: {tempo_txt(t['inicio_s'])} a "
                  f"{tempo_txt(t['fim_s'])}, " + ", ".join(f"câmara {c}: {k} imagens"
                                                         for c, k in t["imagens_por_camara"].items()))

    def alternar(cam):
        """Liga / desliga a deteção da câmara da direita (a da esquerda fica sempre ligada)."""
        if cam != direita:
            return
        agora = time.time() - t0_wall
        ativa[cam] = not ativa[cam]
        if ativa[cam]:
            desligada[cam].append((desligada_desde[cam], agora))
            desligada_desde[cam] = None
            no_ros.ultimo_frame[cam] = None        # recomeça na próxima imagem (a mais recente)
            t_ant[cam] = None
            print(f"[{tempo_txt(agora)}] Câmara {cam}: deteção LIGADA")
        else:
            desligada_desde[cam] = agora
            print(f"[{tempo_txt(agora)}] Câmara {cam}: deteção DESLIGADA "
                  f"(a câmara {', '.join(c for c in camaras if ativa[c])} passa a ser processada mais vezes; "
                  "o vídeo desta câmara deixa de ser gravado nos trechos)")
        publicar_estado()

    publicar_estado()
    print(f"Sessão: {sessao}  (gravação só nos trechos: {gravador.caminho})")
    print(f"A aguardar imagens da(s) câmara(s) {', '.join(camaras)} "
          f"({'sem interface' if sem_interface else 'com interface'})...")
    print("--- CONTROLOS ---")
    print("[R]      : Começar / terminar um trecho (só nos trechos se grava o vídeo original e as deteções)"
          + ("  - sem janela: bash trecho.sh, noutro terminal" if sem_interface else ""))
    if direita:
        print(f"[D]      : Desligar / voltar a ligar a deteção da câmara {direita} (direita)"
              + ("  - sem janela: bash alternar_camara.sh, noutro terminal" if sem_interface else ""))
    print("Ctrl + C : Terminar" if sem_interface else "[Q] [Esc]: Terminar")
    if registar_tudo:
        alternar_trecho()
    else:
        print("A deteção já está a correr; nada é gravado até começar um trecho.")

    try:
        while rclpy.ok():
            # comandos recebidos pelo tópico (alternar_camara.sh)
            while no_ros.comandos:
                cmd = no_ros.comandos.popleft()
                if cmd in ("alternar", "alternar_direita") and direita:
                    alternar(direita)
                elif cmd == "trecho":
                    alternar_trecho()

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

                # Publicar as deteções (mesmo formato do registo offline, sem os contornos da estrada)
                t_img = stamp_ns(msg)
                dados = registo.dados_deteccoes(caixas, linhas_frame, mascara_estrada, t_img,
                                                corrigida=corrigida, camara=cam, contornos=False)
                texto = no_ros.publicar_dados(cam, dados)
                trecho = gravador.deteccoes(cam, texto, t_img)    # gravada só dentro de um trecho
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

                # Desenho (só com janela, ou se alguém subscrever a imagem anotada)
                anotada = no_ros.quer_imagem_anotada(cam)
                if not sem_interface or anotada:
                    visual.desenhar_estrada(frame, mascara_estrada)
                    visual.desenhar_linhas(frame, linhas_frame)
                    visual.desenhar_caixas(frame, caixas)
                    estado_cal = ("corrigida" if corrigida else
                                  "sem correção" if sem_correcao else "sem calibração")
                    n_obj, n_lin = len(caixas), len(linhas_frame)
                    valores = {"objetos": f"{n_obj} objeto" + ("" if n_obj == 1 else "s"),
                               "linhas": f"{n_lin} linha" + ("" if n_lin == 1 else "s"),
                               "estrada": f"{mascara_estrada.mean():.0%} da imagem"
                               if mascara_estrada is not None else ""}
                    info = {"tempo": time.time() - t0_wall, "total": 0,
                            "subtitulo": f"Câmara {cam}  ·  tempo real  ·  {estado_cal}",
                            "cor_subtitulo": "#9CA3AF" if corrigida else "#F59E0B",
                            "modelos": [(k, nome_modelo[k], valores[k], tempos[k], True, True) for k in MODELOS],
                            "rodape": f"{freq[cam] or 0:.1f} deteções/s nesta câmara",
                            # indicador de gravação: um só, na câmara mais à direita que está ligada
                            "registo": texto_registo()
                            if cam == [c for c in camaras if ativa[c]][-1] else None}
                    ecra[cam] = visual.compor(frame, caixas, info)
                    if anotada:
                        no_ros.publicar_imagem(cam, ecra[cam], msg)
                soma_ms["total"] += (time.perf_counter() - t_inicio) * 1000

            if not sem_interface:
                if processou:
                    ref = next((im for im in ecra.values() if im is not None), None)
                    if ref is not None:
                        h, w = ref.shape[:2]
                        partes = []
                        for c in camaras:
                            if not ativa[c]:
                                partes.append(visual.imagem_espera(
                                    w, h, f"Câmara {c}: deteção desligada",
                                    "tecla D para voltar a ligar (enquanto desligada, o vídeo desta câmara não é gravado)"))
                            elif ecra[c] is None:
                                partes.append(visual.imagem_espera(w, h, f"Câmara {c}: à espera da primeira imagem"))
                            else:
                                partes.append(ecra[c])
                        juntas = visual.lado_a_lado(partes)
                        if direita and not janela_ajustada:
                            cv2.resizeWindow(JANELA, juntas.shape[1], juntas.shape[0])
                            janela_ajustada = True
                        cv2.imshow(JANELA, juntas)
                tecla = cv2.waitKey(1) & 0xFF
                if tecla in (ord('q'), 27):
                    break
                if tecla == ord('d') and direita:
                    alternar(direita)
                if tecla == ord('r'):
                    alternar_trecho()
            elif not processou:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        fim = time.time() - t0_wall
        for c in camaras:
            if desligada_desde[c] is not None:
                desligada[c].append((desligada_desde[c], fim))
        trechos = []
        try:
            trechos = gravador.fechar()
            mostrar_trechos(trechos, gravador)
        except Exception as e:
            print(f"Aviso: a gravação não terminou bem ({type(e).__name__}: {e}); o bag pode ficar sem "
                  "índice (o offline lê-o na mesma).")
        try:
            gravar_resumo(sessao, inicio, fim, camaras, corretor, origem_calib, sem_correcao, sem_interface,
                          n_img, desligada, soma_ms, detetor_objetos, detetor_linhas, detetor_estrada,
                          nome_modelo, resumo_det, trechos, gravador)
        except Exception as e:
            print(f"Aviso: não foi possível gravar o resumo ({type(e).__name__}: {e}).")
        if not sem_interface:
            cv2.destroyAllWindows()
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
                  trechos=(), gravador=None):
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
        "sessao": sessao,
        "inicio": inicio.isoformat(timespec="seconds"),
        "duracao_s": round(duracao_s, 1),
        "com_interface": not sem_interface,
        "camaras": [{"camara": c,
                     "topico_imagem": f"/camara{c}/image_raw",
                     "topico_deteccoes": registo.topico_deteccoes(c),
                     "correcao": origem_calib[c] if c in corretor else
                     ("desligada (--sem-correcao)" if sem_correcao else None),
                     "imagens_processadas": n_img[c],
                     "deteccoes_por_segundo": round(n_img[c] / ativa_s[c], 2) if ativa_s[c] > 0 else None,
                     "desligada_s": [[round(a, 1), round(b, 1)] for a, b in desligada[c]]} for c in camaras],
        "modelos": {
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
                     "inicio_ns": gravador.inicio_ns if gravador else None},
        "trechos": [_trecho_para_json(t, gravador) for t in trechos],
        "resumos": resumos,                         # a sessão inteira (como antes)
        "resumos_por_filtro": resumos_por_filtro,   # "sessao", "trechos" e "1", "2", ... (cada trecho)
    }
    pasta = PASTA_REGISTO / sessao
    pasta.mkdir(parents=True, exist_ok=True)
    caminho = pasta / "resumo_online.json"
    caminho.write_text(json.dumps(resumo, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nDuração: {tempo_txt(duracao_s)}  ·  imagens processadas: "
          + ", ".join(f"câmara {c}: {n_img[c]}" for c in camaras))
    print(f"Resumo gravado em: {caminho}")


if __name__ == '__main__':
    main()
