# -*- coding: utf-8 -*-
"""
registo.py - Registo das deteções em ficheiro, e leitura desse registo.

Formato igual ao do processamento em tempo real (branch novo_docker, processar_tempo_real.py),
para os registos online e offline poderem ser lidos da mesma forma:
  * um ficheiro MCAP (ROS 2) com uma mensagem por imagem processada;
  * tópico /camara<N>/processamento/dados_deteccoes, tipo std_msgs/msg/String;
  * o texto da mensagem é um JSON com os campos
      "objetos":  [{"classe": "car", "confianca": 0.84, "caixa": [x1, y1, x2, y2]}, ...]
      "linhas":   [[[x, y], [x, y], ...], ...]          (pontos de cada linha, em píxeis)
      "estrada_percentagem": 0.22                        (fração da imagem que é área circulável, 0 a 1)

Campos acrescentados no registo offline (quem lê só os três campos acima não é afetado):
      "camara":            câmara da imagem ("5" ou "8"; também está no nome do tópico);
      "stamp_ns":          instante da imagem no bag original (ns), para a encontrar depois;
      "trecho":            número do trecho (1, 2, ...) a que a imagem pertence;
      "corrigida":         se a imagem estava corrigida da distorção (as coordenadas são dessa imagem);
      "modelos_ativos":    {"objetos": true, "linhas": true, "estrada": false} (um modelo desligado
                           com as teclas O/L/E aparece com lista vazia, mas assim sabe-se que não correu);
      "estrada_contornos": contornos da área circulável ([[x, y], ...] por zona), para a desenhar depois;
      "estrada_buracos":   contornos de zonas dentro da área que não são estrada (ex. um carro no meio).
O instante de cada mensagem no MCAP (log_time) é também o da imagem original.

Cada processamento com registo cria uma pasta (um "ensaio") com:
  deteccoes.mcap   as mensagens de todos os trechos registados;
  ensaio.json      bag de origem, câmara(s), calibração usada, modelos e parâmetros, e a lista de
                   trechos (início e fim de cada um), para se poder refazer o vídeo de qualquer trecho.

Duas câmaras: o ensaio é um só (os trechos são os mesmos para as duas) e o deteccoes.mcap tem um
tópico por câmara (/camara5/processamento/dados_deteccoes e /camara8/...), como no tempo real.
O ensaio.json tem a lista "camaras", com o tópico da imagem, o tópico das deteções, a calibração e
o tamanho da imagem de cada câmara (camaras_do_ensaio lê também os ensaios antigos, de uma câmara).
"""
import bisect
import re
import datetime
import json
from pathlib import Path

import cv2
import numpy as np

TIPO_MENSAGEM = "std_msgs/msg/String"
DEFINICAO_MENSAGEM = "string data"
VERSAO_FORMATO = 2      # 2: lista "camaras" no ensaio.json (uma ou duas câmaras no mesmo ensaio)


def topico_deteccoes(camara):
    """Igual ao da branch novo_docker."""
    return f"/camara{camara}/processamento/dados_deteccoes" if camara else "/processamento/dados_deteccoes"


def camara_do_topico(topico):
    """'/camara5/processamento/dados_deteccoes' -> '5' (None se o tópico não tiver câmara)."""
    m = re.match(r"/camara([0-9A-Za-z]+)/", topico)
    return m.group(1) if m else None


def contornos_da_mascara(mascara, area_minima=200, tolerancia_px=1.5):
    """Máscara booleana -> (contornos exteriores, buracos), cada um uma lista de [[x, y], ...]."""
    if mascara is None or not mascara.any():
        return [], []
    contornos, hierarquia = cv2.findContours(mascara.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    exteriores, buracos = [], []
    for c, h in zip(contornos, hierarquia[0]):
        if cv2.contourArea(c) < area_minima:
            continue
        c = cv2.approxPolyDP(c, tolerancia_px, True).reshape(-1, 2).tolist()
        (buracos if h[3] >= 0 else exteriores).append(c)   # com pai = buraco dentro de uma zona
    return exteriores, buracos


def mascara_dos_contornos(exteriores, buracos, largura, altura):
    """Inverso de contornos_da_mascara (para desenhar a área circulável a partir do registo)."""
    m = np.zeros((altura, largura), dtype=np.uint8)
    if exteriores:
        cv2.fillPoly(m, [np.array(c, dtype=np.int32) for c in exteriores], 1)
    if buracos:
        cv2.fillPoly(m, [np.array(c, dtype=np.int32) for c in buracos], 0)
    return m.astype(bool)


def dados_deteccoes(caixas, linhas_pontos, mascara_estrada, stamp_ns, trecho=None, corrigida=None,
                    modelos_ativos=None, camara=None):
    """Mesmos campos (e arredondamentos) do processar_tempo_real.py, mais os campos offline."""
    exteriores, buracos = contornos_da_mascara(mascara_estrada)
    return {
        "objetos": [
            {"classe": c, "confianca": round(p, 2), "caixa": [int(x1), int(y1), int(x2), int(y2)]}
            for (x1, y1, x2, y2, c, p) in caixas
        ],
        "linhas": [[[round(float(pt[0]), 1), round(float(pt[1]), 1)] for pt in linha] for linha in linhas_pontos],
        "estrada_percentagem": float(mascara_estrada.mean()) if mascara_estrada is not None else 0.0,
        "stamp_ns": int(stamp_ns),
        "camara": camara,
        "trecho": trecho,
        "corrigida": corrigida,
        "modelos_ativos": modelos_ativos,
        "estrada_contornos": exteriores,
        "estrada_buracos": buracos,
    }


class Registador:
    """
    Escreve o registo de um ensaio. A pasta só é criada quando começa o primeiro trecho.
    Uso: iniciar_trecho(t) -> escrever(t, dados, camara) por imagem -> terminar_trecho() -> ... -> fechar().
    camaras: lista das câmaras do ensaio (ou uma só câmara, ou None); com duas, os trechos são comuns
    e cada câmara escreve no seu tópico.
    """

    def __init__(self, pasta, camaras, metadados):
        self.pasta = Path(pasta)
        if camaras is None or isinstance(camaras, str):
            camaras = [camaras]
        self.camaras = list(camaras)
        self.camara = self.camaras[0]
        self.topicos = {c: topico_deteccoes(c) for c in self.camaras}
        self.topico = self.topicos[self.camara]
        self.metadados = dict(metadados)
        self.trechos = []
        self.trecho = None          # trecho em curso (dict) ou None
        self._ficheiro = None
        self._escritor = None
        self._esquema = None

    @property
    def a_registar(self):
        return self.trecho is not None

    def _abrir(self):
        from mcap.writer import CompressionType
        from mcap_ros2.writer import Writer
        base, n = self.pasta, 2
        while self.pasta.exists():          # nunca escreve por cima de outro ensaio
            self.pasta = base.with_name(f"{base.name}_{n}")
            n += 1
        self.pasta.mkdir(parents=True)
        self._ficheiro = open(self.pasta / "deteccoes.mcap", "wb")
        # sem compressão, como o "ros2 bag record" por omissão (o registo é pequeno)
        self._escritor = Writer(self._ficheiro, compression=CompressionType.NONE)
        self._esquema = self._escritor.register_msgdef(TIPO_MENSAGEM, DEFINICAO_MENSAGEM)

    def iniciar_trecho(self, extra=None):
        if self.a_registar:
            return
        if self._escritor is None:
            self._abrir()
        self.trecho = {"n": len(self.trechos) + 1, "inicio_ns": None, "fim_ns": None, "imagens": 0,
                       "imagens_por_camara": {str(c): 0 for c in self.camaras}}
        if extra:
            self.trecho.update(extra)

    def escrever(self, t_ns, dados, camara=None):
        if not self.a_registar:
            return
        camara = self.camara if camara is None else camara
        if self.trecho["inicio_ns"] is None:
            self.trecho["inicio_ns"] = int(t_ns)
        self.trecho["inicio_ns"] = min(self.trecho["inicio_ns"], int(t_ns))
        self.trecho["fim_ns"] = max(self.trecho["fim_ns"] or 0, int(t_ns))
        self.trecho["imagens"] += 1
        self.trecho["imagens_por_camara"][str(camara)] = self.trecho["imagens_por_camara"].get(str(camara), 0) + 1
        self._escritor.write_message(self.topicos[camara], self._esquema,
                                     {"data": json.dumps(dados, separators=(",", ":"))},
                                     log_time=int(t_ns), publish_time=int(t_ns))

    def terminar_trecho(self):
        """Fecha o trecho em curso; devolve-o (ou None se não tinha imagens)."""
        if not self.a_registar:
            return None
        trecho, self.trecho = self.trecho, None
        if trecho["imagens"] == 0:
            return None
        inicio_bag = self.metadados.get("bag", {}).get("inicio_ns")
        if inicio_bag:
            trecho["inicio_s"] = round((trecho["inicio_ns"] - inicio_bag) / 1e9, 3)
            trecho["fim_s"] = round((trecho["fim_ns"] - inicio_bag) / 1e9, 3)
        self.trechos.append(trecho)
        self._guardar_ensaio()      # guardado a cada trecho, para não se perder nada se o programa parar
        return trecho

    def _guardar_ensaio(self):
        ensaio = {
            "formato": {"versao": VERSAO_FORMATO, "tipo_mensagem": TIPO_MENSAGEM,
                        "nota": "mesmo tópico, tipo e campos do processar_tempo_real.py (branch novo_docker); "
                                "campos extra: stamp_ns, trecho, corrigida, modelos_ativos, "
                                "estrada_contornos, estrada_buracos"},
            "atualizado_em": datetime.datetime.now().isoformat(timespec="seconds"),
            # campos de uma câmara (como nos ensaios antigos); com duas, ver "camaras"
            "camara": self.camara if len(self.camaras) == 1 else None,
            "topico_deteccoes": self.topico if len(self.camaras) == 1 else None,
            "ficheiro_deteccoes": "deteccoes.mcap",
            **self.metadados,
            "trechos": self.trechos,
        }
        (self.pasta / "ensaio.json").write_text(json.dumps(ensaio, indent=2, ensure_ascii=False), encoding="utf-8")

    def fechar(self):
        self.terminar_trecho()
        if self._escritor is not None:
            self._escritor.finish()
            self._ficheiro.close()
            self._escritor = None
            self._guardar_ensaio()


# --------------------------------------------------------------------------- #
# Leitura
# --------------------------------------------------------------------------- #
def pasta_do_ensaio(caminho):
    """Aceita a pasta do ensaio, o ensaio.json ou o deteccoes.mcap; devolve a pasta."""
    caminho = Path(caminho)
    return caminho.parent if caminho.is_file() else caminho


def ler_ensaio(pasta):
    caminho = pasta_do_ensaio(pasta) / "ensaio.json"
    if not caminho.exists():
        return None
    return json.loads(caminho.read_text(encoding="utf-8"))


def camaras_do_ensaio(ensaio):
    """
    Câmaras de um ensaio, pela ordem do ecrã (5 à esquerda, 8 à direita): lista de dicionários com
    camara, topico_imagem, topico_deteccoes, calibracao (dict ou None) e imagem ({largura, altura}).
    Funciona com os ensaios novos (lista "camaras") e com os antigos (uma câmara, campos no topo).
    """
    if ensaio.get("camaras"):
        cams = [dict(c) for c in ensaio["camaras"]]
    else:
        c = ensaio.get("camara")
        cams = [{"camara": c, "topico_imagem": ensaio.get("bag", {}).get("topico_imagem"),
                 "topico_deteccoes": ensaio.get("topico_deteccoes") or topico_deteccoes(c),
                 "calibracao": ensaio.get("calibracao"), "imagem": ensaio.get("imagem")}]
    import visual
    ordem = visual.ordenar_camaras([c["camara"] for c in cams])
    return sorted(cams, key=lambda c: ordem.index(c["camara"]))


def texto_camaras(camaras):
    """['5', '8'] -> '5 e 8'."""
    camaras = [str(c) for c in camaras if c is not None]
    if not camaras:
        return "—"
    return camaras[0] if len(camaras) == 1 else ", ".join(camaras[:-1]) + " e " + camaras[-1]


def listar_ensaios(base):
    """Todas as pastas de ensaio (com ensaio.json) dentro de base, da mais antiga para a mais recente."""
    return sorted((p.parent for p in Path(base).rglob("ensaio.json")), key=lambda p: p.stat().st_mtime)


def ler_trechos_texto(texto):
    """'10-20, 35.5-50, 60-' -> [(10.0, 20.0), (35.5, 50.0), (60.0, inf)] (segundos desde o início do bag)."""
    trechos = []
    for parte in texto.split(","):
        parte = parte.strip()
        if not parte:
            continue
        if "-" not in parte:
            raise ValueError(f"trecho '{parte}' sem '-' (use início-fim, em segundos, ex. 10-20)")
        a, b = parte.split("-", 1)
        a = float(a) if a.strip() else 0.0
        b = float(b) if b.strip() else float("inf")
        if b <= a:
            raise ValueError(f"trecho '{parte}': o fim tem de ser maior do que o início")
        trechos.append((a, b))
    return sorted(trechos)


def ler_deteccoes(caminho_mcap):
    """
    {câmara: {instante_ns: dados}} de todas as mensagens .../processamento/dados_deteccoes de um MCAP
    (registo offline ou do processamento em tempo real, com uma ou duas câmaras). A câmara vem do
    tópico (/camara5/...); num tópico sem câmara, a chave é None. Lê também ficheiros sem índice.
    """
    from mcap.reader import NonSeekingReader
    from mcap_ros2.decoder import DecoderFactory
    from mcap_ros2.reader import read_ros2_messages
    deteccoes = {}
    with open(caminho_mcap, "rb") as f:
        leitor = NonSeekingReader(f, decoder_factories=[DecoderFactory()])
        try:
            for m in read_ros2_messages(leitor, log_time_order=False):
                if not m.channel.topic.endswith("/processamento/dados_deteccoes"):
                    continue
                dados = json.loads(m.ros_msg.data)
                camara = camara_do_topico(m.channel.topic) or dados.get("camara")
                deteccoes.setdefault(camara, {})[int(dados.get("stamp_ns", m.log_time_ns))] = dados
        except Exception as e:
            lidas = sum(len(d) for d in deteccoes.values())
            print(f"Aviso: o registo está incompleto ({type(e).__name__}); foram lidas {lidas} mensagens.")
    return deteccoes


# --------------------------------------------------------------------------- #
# Estatísticas (resumo de um registo, ou de um trecho)
# --------------------------------------------------------------------------- #
VEICULOS = ("car", "motorcycle", "bicycle", "bus", "truck")   # classes do COCO contadas como veículos


def n_veiculos(dados):
    return sum(1 for o in dados.get("objetos", []) if o["classe"] in VEICULOS)


def veiculos_por_instante(por_camara, inicio_ns=None, fim_ns=None, tolerancia_ns=100_000_000):
    """
    Veículos (VEICULOS) em cada instante, somando as câmaras: [(instante_ns, n), ...].
    por_camara: {câmara: {instante_ns: dados}} (só as câmaras a considerar). Com uma câmara, é o
    número de veículos de cada imagem. Com duas, cada imagem da câmara com mais imagens é somada com
    a imagem da outra câmara mais próxima no tempo (até tolerancia_ns de diferença; sem nenhuma tão
    perto, essa câmara não conta nesse instante). As imagens de outra câmara sem nenhuma imagem da
    câmara de referência por perto (por exemplo, num registo juntado em que só essa câmara foi
    registada nesse intervalo) contam como instantes só dessa câmara. Só entram imagens com os
    objetos ligados e, se dados, entre inicio_ns (inclusive) e fim_ns (exclusive).
    """
    def dentro(t):
        return (inicio_ns is None or t >= inicio_ns) and (fim_ns is None or t < fim_ns)

    listas = {}
    for cam, det in por_camara.items():
        ts = sorted(t for t, d in det.items() if dentro(t)
                    and (d.get("modelos_ativos") or {}).get("objetos", True))
        listas[cam] = (ts, det)
    if not listas:
        return []
    ref = max(listas, key=lambda c: len(listas[c][0]))
    resultado = []
    for t in listas[ref][0]:
        n = n_veiculos(listas[ref][1][t])
        for cam, (ts, det) in listas.items():
            if cam == ref or not ts:
                continue
            i = bisect.bisect_left(ts, t)
            vizinhos = [ts[j] for j in (i - 1, i) if 0 <= j < len(ts)]
            mais_perto = min(vizinhos, key=lambda x: abs(x - t))
            if abs(mais_perto - t) <= tolerancia_ns:
                n += n_veiculos(det[mais_perto])
        resultado.append((t, n))
    ts_ref = listas[ref][0]
    for cam, (ts, det) in listas.items():
        if cam == ref:
            continue
        for t in ts:
            i = bisect.bisect_left(ts_ref, t)
            if all(abs(ts_ref[j] - t) > tolerancia_ns for j in (i - 1, i) if 0 <= j < len(ts_ref)):
                resultado.append((t, n_veiculos(det[t])))
    return sorted(resultado)


def estatisticas(lista_dados, gamas=None):
    """
    Resumo de uma lista de mensagens de deteções (as de um trecho, ou de vários):
      imagens; por classe do YOLO: deteções (somadas imagem a imagem), imagens com pelo menos uma,
      máximo numa só imagem e confiança média; linhas: média por imagem e % de imagens sem linhas;
      estrada: % média, mínima e máxima da imagem; veículos por imagem: média, máximo e, se forem
      dadas gamas [(mín, máx, rótulo), ...], a % de imagens em cada gama.
    Só entram nas médias de cada modelo as imagens em que esse modelo estava ligado.
    """
    n = len(lista_dados)
    classes, n_obj = {}, 0
    linhas_por_img, estrada_pct, veic = [], [], []
    for d in lista_dados:
        ativos = d.get("modelos_ativos") or {"objetos": True, "linhas": True, "estrada": True}
        if ativos.get("objetos", True):
            n_obj += 1
            contagem = {}
            for o in d.get("objetos", []):
                c = classes.setdefault(o["classe"], {"deteccoes": 0, "imagens": 0, "maximo": 0, "soma_conf": 0.0})
                c["deteccoes"] += 1
                c["soma_conf"] += o["confianca"]
                contagem[o["classe"]] = contagem.get(o["classe"], 0) + 1
            for nome, k in contagem.items():
                classes[nome]["imagens"] += 1
                classes[nome]["maximo"] = max(classes[nome]["maximo"], k)
            veic.append(n_veiculos(d))
        if ativos.get("linhas", True):
            linhas_por_img.append(len(d.get("linhas", [])))
        if ativos.get("estrada", True):
            estrada_pct.append(float(d.get("estrada_percentagem", 0.0)))
    for c in classes.values():
        c["confianca_media"] = c.pop("soma_conf") / c["deteccoes"] if c["deteccoes"] else 0.0
        c["media_por_imagem"] = c["deteccoes"] / n_obj if n_obj else 0.0
        c["pct_imagens"] = c["imagens"] / n_obj if n_obj else 0.0
    res = {
        "imagens": n,
        "objetos": {"imagens": n_obj, "classes": dict(sorted(classes.items(), key=lambda kv: -kv[1]["deteccoes"]))},
        "linhas": {"imagens": len(linhas_por_img),
                   "media": float(np.mean(linhas_por_img)) if linhas_por_img else None,
                   "pct_sem_linhas": (sum(1 for x in linhas_por_img if x == 0) / len(linhas_por_img))
                   if linhas_por_img else None},
        "estrada": {"imagens": len(estrada_pct),
                    "media": float(np.mean(estrada_pct)) if estrada_pct else None,
                    "minimo": float(np.min(estrada_pct)) if estrada_pct else None,
                    "maximo": float(np.max(estrada_pct)) if estrada_pct else None},
        "veiculos": {"media": float(np.mean(veic)) if veic else None, "maximo": max(veic) if veic else None},
    }
    if gamas and veic:
        res["veiculos"]["gamas"] = [(rotulo, sum(1 for v in veic if a <= v <= b) / len(veic)) for a, b, rotulo in gamas]
    return res
