# -*- coding: utf-8 -*-
"""
gravacao_online.py - Gravação do modo online por trechos (usado pelo processar_tempo_real.py).

A deteção corre sempre; o vídeo original e as deteções só são gravados enquanto um trecho estiver
ativo (tecla R na janela, bash scripts/trecho.sh sem janela, ou --registar-tudo). Grava um bag MCAP igual ao
que o "ros2 bag record" gravava (mesmos tópicos, tipos e formato), mas só com os trechos:

  /camara<N>/image_raw/compressed             sensor_msgs/msg/CompressedImage  vídeo original em JPEG
                                                                     (só nos trechos e só das câmaras com
                                                                     deteção ligada)
  /camara<N>/processamento/dados_deteccoes    std_msgs/msg/String    JSON das deteções (só nos trechos)
  /processamento/estado_camaras               std_msgs/msg/String    câmaras ligadas, a cada mudança
  /processamento/trechos                      std_msgs/msg/String    início e fim de cada trecho (e da sessão)
  /<grupo>/grelha_local                       nav_msgs/msg/OccupancyGrid  grelha local (só nos trechos)
  /tf_static                                  tf2_msgs/msg/TFMessage posição das câmaras no carro (no início)
  /<grupo>/aviso_faixa                        std_msgs/msg/String    aviso de mudança de faixa (JSON, nos trechos)
  /<grupo>/aviso_saida_faixa                  std_msgs/msg/String    aviso de saída de faixa (JSON, nos trechos)
  /<grupo>/aviso_colisao                      std_msgs/msg/String    aviso de colisão (JSON, nos trechos)
  /<grupo>/estado, caminho_local, zona_proibida  saídas do planeador A* comum (nos trechos)

* As imagens são gravadas comprimidas (JPEG), como pede o enunciado (passo 4): o driver da câmara
  (usb_cam, com o compressed_image_transport instalado no Dockerfile) publica também
  /camara<N>/image_raw/compressed, e essas mensagens são gravadas tal como chegam (os bytes da
  mensagem ROS, sem descodificar nem voltar a codificar). Cerca de 4 a 8 vezes menos espaço do que a
  imagem sem compressão. Se esse tópico não chegar (plugin em falta), a imagem sem compressão é
  comprimida aqui, na thread da escrita, com a mesma qualidade (95) e no mesmo formato.
  Em ambos os casos o instante no bag é o da imagem (header.stamp). É esse instante que liga cada
  deteção (stamp_ns) à sua imagem, como no offline.
* O evento "sessao_inicio" leva também o estado do relógio do PC (relogio.py): se estava sincronizado
  por NTP e o desvio medido.
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
DEFINICAO_COMPRIMIDA = """std_msgs/Header header
string format
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
_SEP = "=" * 80 + "\n"
_HEADER = ("MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n" + _SEP
           + "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n")
_QUATERNION = "MSG: geometry_msgs/Quaternion\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n"
DEFINICAO_GRELHA = ("std_msgs/Header header\nnav_msgs/MapMetaData info\nint8[] data\n" + _SEP + _HEADER + _SEP
                    + "MSG: nav_msgs/MapMetaData\nbuiltin_interfaces/Time map_load_time\nfloat32 resolution\n"
                      "uint32 width\nuint32 height\ngeometry_msgs/Pose origin\n" + _SEP
                    + "MSG: geometry_msgs/Pose\ngeometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\n"
                    + _SEP + "MSG: geometry_msgs/Point\nfloat64 x\nfloat64 y\nfloat64 z\n" + _SEP + _QUATERNION)
DEFINICAO_CAMINHO = ("std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses\n" + _SEP + _HEADER + _SEP
                     + "MSG: geometry_msgs/PoseStamped\nstd_msgs/Header header\ngeometry_msgs/Pose pose\n" + _SEP
                     + "MSG: geometry_msgs/Pose\ngeometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\n"
                     + _SEP + "MSG: geometry_msgs/Point\nfloat64 x\nfloat64 y\nfloat64 z\n" + _SEP + _QUATERNION)
DEFINICAO_TF = ("geometry_msgs/TransformStamped[] transforms\n" + _SEP
                + "MSG: geometry_msgs/TransformStamped\nstd_msgs/Header header\nstring child_frame_id\n"
                  "geometry_msgs/Transform transform\n" + _SEP + _HEADER + _SEP
                + "MSG: geometry_msgs/Transform\ngeometry_msgs/Vector3 translation\ngeometry_msgs/Quaternion rotation\n"
                + _SEP + "MSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n" + _SEP + _QUATERNION)
MARGEM_INICIO_NS = 500_000_000      # o "inicio" fica 0,5 s antes do pedido (imagens já captadas e ainda a chegar)
QUALIDADE_JPEG = 95                 # a mesma do compressed_image_transport (por omissão)
FORMATO_JPEG = "bgr8; jpeg compressed bgr8"
ESPERA_COMPRIMIDA_S = 1.0           # sem imagem comprimida do driver há mais do que isto: comprime-se aqui


def topico_imagem(cam):
    return f"/camara{cam}/image_raw/compressed"


def stamp_da_imagem(dados):
    """header.stamp (ns) de uma sensor_msgs/Image serializada (CDR), sem a descodificar."""
    endian = "<" if dados[1] == 1 else ">"           # cabeçalho CDR: 00 01 = little endian
    sec, nanosec = struct.unpack_from(endian + "iI", dados, 4)
    return sec * 1_000_000_000 + nanosec


class _Leitor:
    """Leitura de CDR (little endian) com o alinhamento das mensagens ROS 2."""

    def __init__(self, dados):
        self.d, self.o = memoryview(dados), 4                 # depois do cabeçalho de 4 bytes

    def _alinhar(self, n):
        self.o += (-(self.o - 4)) % n

    def u32(self):
        self._alinhar(4)
        v = struct.unpack_from("<I", self.d, self.o)[0]
        self.o += 4
        return v

    def i32(self):
        self._alinhar(4)
        v = struct.unpack_from("<i", self.d, self.o)[0]
        self.o += 4
        return v

    def u8(self):
        v = self.d[self.o]
        self.o += 1
        return v

    def texto(self):
        n = self.u32()
        v = bytes(self.d[self.o:self.o + n - 1]).decode("utf-8", "replace") if n else ""
        self.o += n
        return v

    def bytes_(self):
        n = self.u32()
        v = self.d[self.o:self.o + n]
        self.o += n
        return v


class ImagemCDR:
    """sensor_msgs/Image lida dos bytes (com os campos que o leitura_bag.imagem_bgr usa)."""

    def __init__(self, dados):
        if dados[1] != 1:
            raise ValueError("CDR big endian não suportado")
        r = _Leitor(dados)
        self.sec, self.nanosec = r.i32(), r.u32()
        self.frame_id = r.texto()
        self.height, self.width = r.u32(), r.u32()
        self.encoding = r.texto()
        self.is_bigendian = r.u8()
        self.step = r.u32()
        self.data = r.bytes_()


class _Escritor:
    def __init__(self):
        self.b = bytearray(b"\x00\x01\x00\x00")

    def _alinhar(self, n):
        self.b += b"\x00" * ((-(len(self.b) - 4)) % n)

    def u32(self, v, fmt="<I"):
        self._alinhar(4)
        self.b += struct.pack(fmt, v)

    def texto(self, t):
        b = t.encode("utf-8") + b"\x00"
        self.u32(len(b))
        self.b += b

    def bytes_(self, b):
        self.u32(len(b))
        self.b += b


def cdr_imagem_comprimida(sec, nanosec, frame_id, formato, dados_jpeg):
    """sensor_msgs/CompressedImage serializada (CDR little endian)."""
    e = _Escritor()
    e.u32(sec, "<i")
    e.u32(nanosec)
    e.texto(frame_id)
    e.texto(formato)
    e.bytes_(bytes(dados_jpeg))
    return bytes(e.b)


def comprimir_imagem(dados_imagem, qualidade=QUALIDADE_JPEG):
    """sensor_msgs/Image serializada -> sensor_msgs/CompressedImage serializada (JPEG), ou None."""
    import cv2
    from . import leitura_bag
    img = ImagemCDR(dados_imagem)
    bgr = leitura_bag.imagem_bgr(img)
    if bgr is None:
        return None
    ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(qualidade)])
    if not ok:
        return None
    return cdr_imagem_comprimida(img.sec, img.nanosec, img.frame_id, FORMATO_JPEG, jpeg.tobytes())


def cdr_string(texto):
    """std_msgs/String serializada (CDR little endian)."""
    b = texto.encode("utf-8") + b"\x00"
    return b"\x00\x01\x00\x00" + struct.pack("<I", len(b)) + b


class GravadorTrechos:
    def __init__(self, caminho, camaras, extra=None, relogio=None):
        """extra: {chave: (tópico, tipo, definição)} de outras mensagens a gravar (grelha, TF);
        relogio: estado do relógio do PC (relogio.py), gravado no evento "sessao_inicio"."""
        from mcap.writer import CompressionType, Writer
        self.caminho = Path(caminho)
        self.caminho.parent.mkdir(parents=True, exist_ok=True)
        self.camaras = [str(c) for c in camaras]
        self._ficheiro = open(self.caminho, "wb")
        self._escritor = Writer(self._ficheiro, compression=CompressionType.NONE)
        self._escritor.start(profile="ros2", library="processar_tempo_real.py")
        esq_img = self._escritor.register_schema("sensor_msgs/msg/CompressedImage", "ros2msg",
                                                 DEFINICAO_COMPRIMIDA.encode())
        esq_txt = self._escritor.register_schema("std_msgs/msg/String", "ros2msg", DEFINICAO_STRING.encode())
        self._canal = {}
        for c in self.camaras:
            self._canal[("img", c)] = self._escritor.register_channel(topico_imagem(c), "cdr", esq_img)
            self._canal[("det", c)] = self._escritor.register_channel(
                f"/camara{c}/processamento/dados_deteccoes", "cdr", esq_txt)
        self._canal["estado"] = self._escritor.register_channel(TOPICO_ESTADO, "cdr", esq_txt)
        self._canal["trechos"] = self._escritor.register_channel(TOPICO_TRECHOS, "cdr", esq_txt)
        for chave, (topico, tipo, definicao) in (extra or {}).items():
            esq = self._escritor.register_schema(tipo, "ros2msg", definicao.encode())
            self._canal[chave] = self._escritor.register_channel(topico, "cdr", esq)

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
        self._ultimo_stamp = {c: 0 for c in self.camaras}          # última imagem gravada (sem repetir)
        self._ultima_comprimida = {c: None for c in self.camaras}   # quando chegou a última do driver
        self.comprimidas_aqui = {c: 0 for c in self.camaras}        # imagens comprimidas por este programa
        self.relogio = relogio
        ev = {"evento": "sessao_inicio", "stamp_ns": self.inicio_ns}
        if relogio is not None:
            ev["relogio"] = relogio
        self._evento(ev, self.inicio_ns)

    # ----------------------------------------------------------------- escrita
    def _escrever(self):
        while True:
            item = self._fila.get()
            if item is None:
                return
            canal, t_ns, dados = item[:3]
            try:
                if len(item) > 3:                    # imagem sem compressão: comprimir aqui (JPEG)
                    dados = comprimir_imagem(dados)
                    if dados is None:
                        continue
                    self.comprimidas_aqui[item[3]] += 1
                self._escritor.add_message(canal, log_time=t_ns, data=dados, publish_time=t_ns)
            except Exception as e:          # disco cheio, etc.: avisa uma vez e continua a deteção
                if self.erro is None:
                    self.erro = e
                    print(f"\nAviso: a gravação falhou ({type(e).__name__}: {e}); a deteção continua.")

    def _evento(self, dados, t_ns, canal="trechos"):
        self._fila.put((self._canal[canal], int(t_ns), cdr_string(json.dumps(dados, separators=(",", ":")))))

    # ---------------------------------------------- chamado pela receção das imagens
    def _aceitar(self, cam, t):
        """Conta a imagem no trecho em curso (com o lock). False se não é para gravar."""
        if self.trecho is None or cam not in self.ativas or t <= self._ultimo_stamp[cam]:
            return False
        self._ultimo_stamp[cam] = t
        tr = self.trecho
        tr["inicio_ns"] = t if tr["inicio_ns"] is None else min(tr["inicio_ns"], t)
        tr["fim_ns"] = t if tr["fim_ns"] is None else max(tr["fim_ns"], t)
        tr["imagens_por_camara"][cam] = tr["imagens_por_camara"].get(cam, 0) + 1
        return True

    def imagem(self, cam, dados):
        """Imagem comprimida do driver (bytes da sensor_msgs/CompressedImage): gravada se houver trecho e
        a câmara estiver ligada."""
        cam = str(cam)
        t = stamp_da_imagem(dados)
        with self._lock:
            self._ultima_comprimida[cam] = time.monotonic()
            if not self._aceitar(cam, t):
                return
        self._fila.put((self._canal[("img", cam)], t, bytes(dados)))

    def imagem_bruta(self, cam, dados):
        """Imagem sem compressão (sensor_msgs/Image): só é gravada (comprimida aqui) se o driver não
        estiver a publicar a versão comprimida."""
        cam = str(cam)
        with self._lock:
            ultima = self._ultima_comprimida[cam]
            if self.trecho is None or (ultima is not None and time.monotonic() - ultima < ESPERA_COMPRIMIDA_S):
                return
            t = stamp_da_imagem(dados)
            if not self._aceitar(cam, t):
                return
        self._fila.put((self._canal[("img", cam)], t, bytes(dados), cam))

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

    def mensagem(self, chave, dados, t_ns, sempre=False):
        """Outra mensagem já serializada (grelha, TF): gravada dentro de um trecho (ou sempre)."""
        if chave not in self._canal or (not sempre and self.trecho is None):
            return
        self._fila.put((self._canal[chave], int(t_ns), bytes(dados)))

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


class GravadorNulo:
    """Sem gravação (reprodução de um bag no Docker: os dados já estão gravados). Mesma interface."""

    def __init__(self, camaras, inicio_ns=None):
        self.camaras = [str(c) for c in camaras]
        self.inicio_ns = inicio_ns or time.time_ns()
        self.trecho = None
        self.trechos = []
        self.caminho = Path("(sem gravação)")
        self.tamanho_bytes = 0
        self.comprimidas_aqui = {c: 0 for c in self.camaras}
        self.relogio = None

    def imagem(self, *a, **k):
        pass

    imagem_bruta = mensagem = imagem

    def deteccoes(self, *a, **k):
        return None

    def estado(self, *a, **k):
        pass

    def iniciar_trecho(self):
        return None

    def terminar_trecho(self):
        return None

    def fechar(self):
        return []

    def em_espera(self):
        return 0
