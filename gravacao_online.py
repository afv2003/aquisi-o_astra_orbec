# -*- coding: utf-8 -*-
"""
gravacao_online.py - Gravação do modo online por trechos (usado pelo processar_tempo_real.py).

A deteção corre sempre; o vídeo original e as deteções só são gravados enquanto um trecho estiver
ativo (tecla R na janela, bash trecho.sh sem janela, ou --registar-tudo). Grava um bag MCAP igual ao
que o "ros2 bag record" gravava (mesmos tópicos, tipos e formato), mas só com os trechos:

  /camara<N>/image_raw                        sensor_msgs/msg/Image  vídeo original (só nos trechos e
                                                                     só das câmaras com deteção ligada)
  /camara<N>/processamento/dados_deteccoes    std_msgs/msg/String    JSON das deteções (só nos trechos)
  /processamento/estado_camaras               std_msgs/msg/String    câmaras ligadas, a cada mudança
  /processamento/trechos                      std_msgs/msg/String    início e fim de cada trecho (e da sessão)

* As imagens são gravadas tal como chegam da câmara (os bytes da mensagem ROS, sem descodificar nem
  voltar a codificar), com o instante da imagem (header.stamp) como instante no bag. É esse instante
  que liga cada deteção (stamp_ns) à sua imagem, como no offline.
* O bag começa no arranque da sessão (primeira mensagem: início da sessão) e acaba no fim; por isso
  os tempos "desde o início do bag" são os tempos da sessão online, também no offline.
* Mensagens de /processamento/trechos (JSON):
    {"evento": "sessao_inicio", "stamp_ns": ...}
    {"evento": "inicio", "trecho": 1, "stamp_ns": ...}
    {"evento": "fim", "trecho": 1, "stamp_ns": ..., "imagens_por_camara": {...}, "intervalos_por_camara": {...}}
    {"evento": "sessao_fim", "stamp_ns": ...}
  Todas as imagens gravadas de um trecho ficam entre o "inicio" e o "fim" desse trecho (o
  processar_bag.py usa-os para refazer os mesmos trechos).
* A escrita no disco é feita numa thread à parte, para não atrasar a deteção nem a receção das imagens.
"""
import json
import queue
import struct
import threading
import time
from pathlib import Path

TOPICO_TRECHOS = "/processamento/trechos"
TOPICO_ESTADO = "/processamento/estado_camaras"

DEFINICAO_IMAGEM = """std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
DEFINICAO_STRING = "string data\n"
MARGEM_INICIO_NS = 500_000_000      # o "inicio" fica 0,5 s antes do pedido (imagens já captadas e ainda a chegar)


def stamp_da_imagem(dados):
    """header.stamp (ns) de uma sensor_msgs/Image serializada (CDR), sem a descodificar."""
    endian = "<" if dados[1] == 1 else ">"           # cabeçalho CDR: 00 01 = little endian
    sec, nanosec = struct.unpack_from(endian + "iI", dados, 4)
    return sec * 1_000_000_000 + nanosec


def cdr_string(texto):
    """std_msgs/String serializada (CDR little endian)."""
    b = texto.encode("utf-8") + b"\x00"
    return b"\x00\x01\x00\x00" + struct.pack("<I", len(b)) + b


class GravadorTrechos:
    def __init__(self, caminho, camaras):
        from mcap.writer import CompressionType, Writer
        self.caminho = Path(caminho)
        self.caminho.parent.mkdir(parents=True, exist_ok=True)
        self.camaras = [str(c) for c in camaras]
        self._ficheiro = open(self.caminho, "wb")
        self._escritor = Writer(self._ficheiro, compression=CompressionType.NONE)
        self._escritor.start(profile="ros2", library="processar_tempo_real.py")
        esq_img = self._escritor.register_schema("sensor_msgs/msg/Image", "ros2msg", DEFINICAO_IMAGEM.encode())
        esq_txt = self._escritor.register_schema("std_msgs/msg/String", "ros2msg", DEFINICAO_STRING.encode())
        self._canal = {}
        for c in self.camaras:
            self._canal[("img", c)] = self._escritor.register_channel(f"/camara{c}/image_raw", "cdr", esq_img)
            self._canal[("det", c)] = self._escritor.register_channel(
                f"/camara{c}/processamento/dados_deteccoes", "cdr", esq_txt)
        self._canal["estado"] = self._escritor.register_channel(TOPICO_ESTADO, "cdr", esq_txt)
        self._canal["trechos"] = self._escritor.register_channel(TOPICO_TRECHOS, "cdr", esq_txt)

        self._fila = queue.Queue()
        self._lock = threading.Lock()
        self._fio = threading.Thread(target=self._escrever, daemon=True)
        self._fio.start()
        self.erro = None

        self.inicio_ns = time.time_ns()
        self.trechos = []                 # trechos já terminados (dicionários, como no ensaio.json)
        self.trecho = None                # trecho em curso
        self.ativas = set(self.camaras)   # câmaras com deteção ligada (só essas são gravadas)
        self._ultimo_fim = 0
        self._ultimo_log = 0
        self._evento({"evento": "sessao_inicio", "stamp_ns": self.inicio_ns}, self.inicio_ns)

    # ----------------------------------------------------------------- escrita
    def _escrever(self):
        while True:
            item = self._fila.get()
            if item is None:
                return
            canal, t_ns, dados = item
            try:
                self._escritor.add_message(canal, log_time=t_ns, data=dados, publish_time=t_ns)
            except Exception as e:          # disco cheio, etc.: avisa uma vez e continua a deteção
                if self.erro is None:
                    self.erro = e
                    print(f"\nAviso: a gravação falhou ({type(e).__name__}: {e}); a deteção continua.")

    def _evento(self, dados, t_ns, canal="trechos"):
        self._fila.put((self._canal[canal], int(t_ns), cdr_string(json.dumps(dados, separators=(",", ":")))))

    # ---------------------------------------------- chamado pela receção das imagens
    def imagem(self, cam, dados):
        """Imagem recebida (bytes da mensagem ROS): gravada se houver trecho e a câmara estiver ligada."""
        cam = str(cam)
        with self._lock:
            if self.trecho is None or cam not in self.ativas:
                return
            t = stamp_da_imagem(dados)
            tr = self.trecho
            tr["inicio_ns"] = t if tr["inicio_ns"] is None else min(tr["inicio_ns"], t)
            tr["fim_ns"] = t if tr["fim_ns"] is None else max(tr["fim_ns"], t)
            tr["imagens_por_camara"][cam] = tr["imagens_por_camara"].get(cam, 0) + 1
        self._fila.put((self._canal[("img", cam)], t, bytes(dados)))

    # ------------------------------------------------- chamado pelo processamento
    def deteccoes(self, cam, texto_json, stamp_ns):
        """Deteções de uma imagem: gravadas se a imagem ficou gravada no trecho em curso.
        Devolve o número do trecho (ou None se não foram gravadas)."""
        cam = str(cam)
        with self._lock:
            tr = self.trecho
            # só as deteções de imagens que ficaram gravadas neste trecho (a imagem pode ter chegado
            # antes de o trecho começar e ter sido processada já depois)
            if tr is None or cam not in self.ativas or tr["inicio_ns"] is None or stamp_ns < tr["inicio_ns"]:
                return None
            tr["deteccoes_por_camara"][cam] = tr["deteccoes_por_camara"].get(cam, 0) + 1
            n = tr["n"]
        self._fila.put((self._canal[("det", cam)], int(stamp_ns), cdr_string(texto_json)))
        return n

    def estado(self, ativas, texto_json):
        """Mudança das câmaras ligadas (tecla D): grava o estado e acerta os intervalos do trecho."""
        agora = time.time_ns()
        with self._lock:
            novas = {str(c) for c in ativas}
            if self.trecho is not None:
                for c in self.camaras:
                    abertos = self.trecho["intervalos_por_camara"][c]
                    if c in novas and c not in self.ativas:
                        abertos.append([agora, None])
                    elif c not in novas and c in self.ativas and abertos and abertos[-1][1] is None:
                        abertos[-1][1] = agora
            self.ativas = novas
        self._evento_estado(texto_json, agora)

    def _evento_estado(self, texto_json, t_ns):
        self._fila.put((self._canal["estado"], int(t_ns), cdr_string(texto_json)))

    @property
    def n_trecho(self):
        return self.trecho["n"] if self.trecho else None

    def iniciar_trecho(self):
        with self._lock:
            if self.trecho is not None:
                return self.trecho["n"]
            agora = time.time_ns()
            t_ev = max(agora - MARGEM_INICIO_NS, self._ultimo_fim + 1, self.inicio_ns + 1)
            n = len(self.trechos) + 1
            self.trecho = {"n": n, "pedido_ns": agora, "evento_inicio_ns": t_ev,
                           "inicio_ns": None, "fim_ns": None,
                           "imagens_por_camara": {c: 0 for c in self.camaras},
                           "deteccoes_por_camara": {c: 0 for c in self.camaras},
                           "intervalos_por_camara": {c: ([[t_ev, None]] if c in self.ativas else [])
                                                     for c in self.camaras}}
        self._evento({"evento": "inicio", "trecho": n, "stamp_ns": t_ev}, t_ev)
        return n

    def terminar_trecho(self):
        """Fecha o trecho em curso; devolve-o (com "imagens" = 0 se não chegou nenhuma imagem)."""
        with self._lock:
            tr, self.trecho = self.trecho, None
            if tr is None:
                return None
            agora = time.time_ns()
            t_ev = max(agora, tr["fim_ns"] or 0) + 1
            for c in self.camaras:
                for intervalo in tr["intervalos_por_camara"][c]:
                    if intervalo[1] is None:
                        intervalo[1] = t_ev
            self._ultimo_fim = t_ev
        tr["evento_fim_ns"] = t_ev
        tr["imagens"] = sum(tr["imagens_por_camara"].values())
        if tr["inicio_ns"] is None:                   # nenhuma imagem: o trecho fica com os instantes do pedido
            tr["inicio_ns"], tr["fim_ns"] = tr["evento_inicio_ns"], t_ev
        tr["inicio_s"] = round((tr["inicio_ns"] - self.inicio_ns) / 1e9, 3)
        tr["fim_s"] = round((tr["fim_ns"] - self.inicio_ns) / 1e9, 3)
        self._evento({"evento": "fim", "trecho": tr["n"], "stamp_ns": t_ev,
                      "imagens_por_camara": tr["imagens_por_camara"],
                      "deteccoes_por_camara": tr["deteccoes_por_camara"],
                      "intervalos_por_camara": tr["intervalos_por_camara"]}, t_ev)
        self.trechos.append(tr)
        return tr

    def fechar(self):
        """Termina o trecho em curso e a sessão; devolve a lista de trechos. Sem nenhuma imagem gravada,
        o ficheiro é apagado (não fica um bag vazio)."""
        if self.trecho is not None:
            self.terminar_trecho()
        fim = max(time.time_ns(), self._ultimo_fim + 1)
        self._evento({"evento": "sessao_fim", "stamp_ns": fim}, fim)
        self._fila.put(None)
        self._fio.join()
        try:
            self._escritor.finish()
        finally:
            self._ficheiro.close()
        if not any(t["imagens"] for t in self.trechos):
            self.caminho.unlink(missing_ok=True)
        return self.trechos

    @property
    def tamanho_bytes(self):
        return self.caminho.stat().st_size if self.caminho.exists() else 0

    def em_espera(self):
        """Mensagens ainda por escrever no disco (se crescer sempre, o disco não acompanha)."""
        return self._fila.qsize()
