# -*- coding: utf-8 -*-
"""
comparacao.py - Partes comuns da comparação de modelos de linhas (comparar_linhas.py e
resumo_comparacao.py): registo das deteções de cada modelo, leitura, concordância e instantes.

Registo: igual ao do processar_bag.py (registo.py, mesmo formato e mesmos trechos), com cada modelo
no lugar de uma câmara: um só deteccoes.mcap com um tópico por modelo,
  /clrnet/processamento/dados_deteccoes   e   /ufld/processamento/dados_deteccoes
com o JSON do registo offline (objetos e estrada vazios: só correm as linhas), mais o campo "modelo".
As duas mensagens de cada imagem têm o mesmo stamp_ns (o instante da imagem no bag, ou no vídeo).

Instantes: em cada pausa, uma pasta instantes/<n>_t<segundos>s/ com as imagens e as deteções
desse instante (ver comparar_linhas.py).
"""
import json
from pathlib import Path

import numpy as np

import registo

MODELOS = ("clrnet", "ufld")                     # ordem no ecrã: CLRNet à esquerda, UFLD à direita
NOMES = {"clrnet": "CLRNet", "ufld": "UFLDv2"}
LIMIAR_PX = 30                                   # distância média máxima (px) para duas linhas serem "a mesma"
PASSO_Y_PX = 10                                  # de quantos em quantos píxeis (em y) se comparam duas linhas


def topico(modelo):
    return f"/{modelo}/processamento/dados_deteccoes"


class RegistadorComparacao(registo.Registador):
    """
    O Registador do offline, com um "tópico por modelo" em vez de um por câmara, e numa pasta fixa
    (criada antes, se se guardar um instante antes do primeiro trecho).
    """

    def __init__(self, pasta, metadados, inicio_ns=0):
        super().__init__(pasta, list(MODELOS), metadados)
        self.topicos = {m: topico(m) for m in MODELOS}
        self.topico = None
        self.inicio_ns = inicio_ns

    def _abrir(self):
        from mcap.writer import CompressionType
        from mcap_ros2.writer import Writer
        self.pasta.mkdir(parents=True, exist_ok=True)
        self._ficheiro = open(self.pasta / "deteccoes.mcap", "wb")
        self._escritor = Writer(self._ficheiro, compression=CompressionType.NONE)
        self._esquema = self._escritor.register_msgdef(registo.TIPO_MENSAGEM, registo.DEFINICAO_MENSAGEM)

    def terminar_trecho(self):
        trecho = super().terminar_trecho()
        if trecho is not None and "inicio_s" not in trecho:      # vídeos: o tempo começa em 0
            trecho["inicio_s"] = round((trecho["inicio_ns"] - self.inicio_ns) / 1e9, 3)
            trecho["fim_s"] = round((trecho["fim_ns"] - self.inicio_ns) / 1e9, 3)
            self._guardar_ensaio()
        return trecho


def ler_deteccoes_modelos(caminho_mcap):
    """{modelo: {stamp_ns: dados}} de um deteccoes.mcap da comparação."""
    from mcap.reader import NonSeekingReader
    from mcap_ros2.decoder import DecoderFactory
    from mcap_ros2.reader import read_ros2_messages
    det = {m: {} for m in MODELOS}
    with open(caminho_mcap, "rb") as f:
        leitor = NonSeekingReader(f, decoder_factories=[DecoderFactory()])
        try:
            for m in read_ros2_messages(leitor, log_time_order=False):
                modelo = m.channel.topic.strip("/").split("/")[0]
                if modelo in det:
                    dados = json.loads(m.ros_msg.data)
                    det[modelo][int(dados.get("stamp_ns", m.log_time_ns))] = dados
        except Exception as e:
            print(f"Aviso: o registo está incompleto ({type(e).__name__}).")
    return det


def pasta_da_sessao(caminho):
    caminho = Path(caminho)
    return caminho.parent if caminho.is_file() else caminho


def listar_sessoes(base):
    """Pastas de sessões da comparação (com ensaio.json ou instantes/), da mais antiga para a mais recente."""
    base = Path(base)
    if not base.exists():
        return []
    pastas = {p.parent for p in base.rglob("ensaio.json")} | {p.parent for p in base.rglob("instantes")}
    return sorted(pastas, key=lambda p: p.stat().st_mtime)


def listar_instantes(pasta_sessao):
    """[(pasta, dados do deteccoes.json)] dos instantes guardados, por ordem."""
    pasta = Path(pasta_sessao) / "instantes"
    if not pasta.exists():
        return []
    res = []
    for p in sorted(pasta.iterdir()):
        f = p / "deteccoes.json"
        if f.exists():
            res.append((p, json.loads(f.read_text(encoding="utf-8"))))
    return res


# --------------------------------------------------------------------------- #
# Concordância entre os dois modelos
# --------------------------------------------------------------------------- #
def _x_em(linha, ys):
    """x da linha em cada y (interpolado); NaN fora da linha."""
    p = np.asarray(linha, dtype=np.float64)
    p = p[np.argsort(p[:, 1])]
    x = np.interp(ys, p[:, 1], p[:, 0])
    x[(ys < p[0, 1]) | (ys > p[-1, 1])] = np.nan
    return x


def distancia(la, lb):
    """Distância média (px, em x) entre duas linhas, na parte em que as duas existem; inf se < 3 pontos."""
    a, b = np.asarray(la, dtype=np.float64), np.asarray(lb, dtype=np.float64)
    y0, y1 = max(a[:, 1].min(), b[:, 1].min()), min(a[:, 1].max(), b[:, 1].max())
    if y1 - y0 < 2 * PASSO_Y_PX:
        return float("inf")
    ys = np.arange(y0, y1 + 1e-6, PASSO_Y_PX)
    d = np.abs(_x_em(a, ys) - _x_em(b, ys))
    d = d[~np.isnan(d)]
    return float(d.mean()) if len(d) >= 3 else float("inf")


def emparelhar(linhas_a, linhas_b, limiar=LIMIAR_PX):
    """Pares (i, j, distância) de linhas que são "a mesma" (as mais próximas primeiro, até ao limiar)."""
    candidatos = sorted((distancia(la, lb), i, j) for i, la in enumerate(linhas_a) for j, lb in enumerate(linhas_b))
    usados_a, usados_b, pares = set(), set(), []
    for d, i, j in candidatos:
        if d > limiar:
            break
        if i not in usados_a and j not in usados_b:
            usados_a.add(i)
            usados_b.add(j)
            pares.append((i, j, d))
    return pares


def concordancia(det_a, det_b, limiar=LIMIAR_PX):
    """
    Concordância entre dois modelos nas imagens que os dois processaram ({stamp_ns: dados} cada):
    imagens, % com o mesmo número de linhas, diferença média do número de linhas, % das linhas de
    cada modelo que o outro também encontrou, distância média entre as linhas correspondentes, e %
    das imagens em que só um dos dois encontrou linhas.
    """
    comuns = sorted(set(det_a) & set(det_b))
    n = len(comuns)
    if not n:
        return {"imagens": 0}
    mesmo_n = dif_n = so_a = so_b = 0
    tot_a = tot_b = emparelhadas = 0
    distancias = []
    for t in comuns:
        la = [l for l in det_a[t].get("linhas", []) if len(l) > 1]
        lb = [l for l in det_b[t].get("linhas", []) if len(l) > 1]
        mesmo_n += len(la) == len(lb)
        dif_n += abs(len(la) - len(lb))
        so_a += bool(la) and not lb
        so_b += bool(lb) and not la
        tot_a += len(la)
        tot_b += len(lb)
        pares = emparelhar(la, lb, limiar)
        emparelhadas += len(pares)
        distancias += [d for _, _, d in pares]
    return {
        "imagens": n,
        "limiar_px": limiar,
        "pct_mesmo_numero": mesmo_n / n,
        "diferenca_media_numero": dif_n / n,
        "linhas_a": tot_a, "linhas_b": tot_b, "emparelhadas": emparelhadas,
        "pct_a_encontradas_por_b": emparelhadas / tot_a if tot_a else None,
        "pct_b_encontradas_por_a": emparelhadas / tot_b if tot_b else None,
        "distancia_media_px": float(np.mean(distancias)) if distancias else None,
        "pct_so_a": so_a / n, "pct_so_b": so_b / n,
    }


def distribuicao_linhas(det, maximo=4):
    """% de imagens com 0, 1, ..., maximo linhas."""
    n = len(det)
    cont = [0] * (maximo + 1)
    for d in det.values():
        cont[min(maximo, len(d.get("linhas", [])))] += 1
    return [c / n if n else 0.0 for c in cont]
