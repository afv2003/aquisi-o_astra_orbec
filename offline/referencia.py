# -*- coding: utf-8 -*-
"""
referencia.py - Conjunto de referência (marcado à mão) para medir o desempenho da deteção.

Usado pelo anotar_referencia.py (escolher as imagens e marcá-las) e pelo metricas_referencia.py
(comparar o processamento com a referência). README, secção 4.11.

Um conjunto tem um nome e duas pastas:
  docs/referencia/<nome>/            (pequeno, pode ir para o git: é a "verdade" do relatório)
      amostra.json                   bags, câmaras, calibração usada e as imagens escolhidas
      anotacoes/<id>.json            o que foi marcado em cada imagem (mesmo formato das deteções)
      metricas.json, metricas.md     os resultados (metricas_referencia.py)
  data/referencia/<nome>/            (imagens; não vai para o git, refaz-se a partir dos bags)
      <id>_original.png              a imagem do bag, sem correção
      <id>_referencia.png, <id>_processamento.png, <id>_comparacao.png   (metricas_referencia.py)

As marcações são feitas na imagem CORRIGIDA da distorção (a que os modelos recebem), com as
coordenadas em píxeis dessa imagem; <id> = <câmara>_<stamp_ns>.
"""
import datetime
import json
import random
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "va_g05"))   # biblioteca comum (pacote va_g05)
from va_g05 import caminhos, leitura_bag, registo, objetos  # noqa: E402

PASTA_REF = caminhos.RAIZ / "docs" / "referencia"
PASTA_IMG = caminhos.DADOS / "referencia"
CLASSES = list(objetos.CLASSES_ESTRADA)            # as classes que o processamento mostra por omissão
ESTADOS_FAIXA = ("LIVRE", "OCUPADA", "SEM_FAIXA")  # aviso de mudança de faixa, do lado da câmara


def pasta(nome):
    return PASTA_REF / nome


def pasta_imagens(nome):
    return PASTA_IMG / nome


def id_imagem(camara, stamp_ns):
    return f"{camara}_{int(stamp_ns)}"


# --------------------------------------------------------------------------- amostra
def _caminho_guardado(bag):
    bag = Path(bag).resolve()
    try:
        return bag.relative_to(caminhos.RAIZ).as_posix()
    except ValueError:
        return str(bag)


def encontrar_bag(info):
    """O bag de uma entrada da amostra: o caminho guardado, ou um com o mesmo nome em data/."""
    p = Path(info["caminho"])
    p = p if p.is_absolute() else caminhos.RAIZ / p
    if p.exists():
        return p
    for c in sorted(caminhos.DADOS.rglob(info["nome"])):
        return c
    return None


def criar_amostra(nome, bags, n, camaras=None, semente=1, intervalo_min_s=2.0):
    """Escolhe n imagens ao acaso (com a semente indicada, para se poder repetir), metade de cada câmara,
    com pelo menos intervalo_min_s entre imagens da mesma câmara (para não serem quase iguais)."""
    candidatas, info_bags, calib = {}, [], {}
    for bag in bags:
        bag = Path(bag)
        erro = leitura_bag.verificar_mcap(bag)
        if erro:
            sys.exit(f"Erro: {bag}: {erro}")
        indice = leitura_bag.bag_tem_indice(bag)
        cams = leitura_bag.camaras_no_bag(leitura_bag.topicos_do_bag(bag) if indice else [])
        if not cams:                                   # bag sem índice: tenta os tópicos habituais
            cams = {c: t for c in ("5", "8") for t in (f"/camara{c}/image_raw/compressed",)}
        cams = {c: t for c, t in cams.items() if not camaras or c in camaras}
        info_bags.append({"nome": bag.name, "caminho": _caminho_guardado(bag), "camaras": cams})
        inicio = None
        for m in leitura_bag.ler_mensagens(bag, list(cams.values()), indice):
            inicio = m.log_time_ns if inicio is None else inicio
            cam = next(c for c, t in cams.items() if t == m.channel.topic)
            candidatas.setdefault(cam, []).append((bag.name, m.log_time_ns, (m.log_time_ns - inicio) / 1e9))
        for cam in cams:
            if cam not in calib:
                ficheiro = caminhos.calibracao(cam)
                if ficheiro.exists():
                    calib[cam] = leitura_bag.calibracao_para_dict(leitura_bag.carregar_calibracao_yaml(ficheiro),
                                                                  f"config/{ficheiro.name}")
    if not candidatas:
        sys.exit("Erro: nenhuma imagem das câmaras nos bags indicados.")
    rng = random.Random(semente)
    escolhidas = []
    cams = sorted(candidatas)
    por_cam = {c: n // len(cams) + (1 if i < n % len(cams) else 0) for i, c in enumerate(cams)}
    for cam in cams:
        lista = candidatas[cam][:]
        rng.shuffle(lista)
        tomadas = []
        for b, t_ns, t_s in lista:
            if len(tomadas) >= por_cam[cam]:
                break
            if all(b != b2 or abs(t_ns - t2) >= intervalo_min_s * 1e9 for b2, t2, _ in tomadas):
                tomadas.append((b, t_ns, t_s))
        escolhidas += [{"id": id_imagem(cam, t_ns), "bag": b, "camara": cam, "stamp_ns": int(t_ns),
                        "t_s": round(t_s, 2)} for b, t_ns, t_s in sorted(tomadas)]
    amostra = {"nome": nome, "criado_em": datetime.datetime.now().isoformat(timespec="seconds"),
               "semente": semente, "intervalo_min_s": intervalo_min_s, "bags": info_bags,
               "calibracao": calib, "corrigida": bool(calib), "imagens": escolhidas}
    pasta(nome).mkdir(parents=True, exist_ok=True)
    (pasta(nome) / "anotacoes").mkdir(exist_ok=True)
    (pasta(nome) / "amostra.json").write_text(json.dumps(amostra, indent=2, ensure_ascii=False), encoding="utf-8")
    return amostra


def ler_amostra(nome):
    f = pasta(nome) / "amostra.json"
    if not f.exists():
        return None
    return json.loads(f.read_text(encoding="utf-8"))


# --------------------------------------------------------------------------- imagens
def extrair_imagens(amostra, so_em_falta=True):
    """Guarda em data/referencia/<nome>/ a imagem original de cada imagem da amostra (lida do bag)."""
    destino = pasta_imagens(amostra["nome"])
    destino.mkdir(parents=True, exist_ok=True)
    por_bag = {}
    for im in amostra["imagens"]:
        if so_em_falta and (destino / f"{im['id']}_original.png").exists():
            continue
        por_bag.setdefault(im["bag"], []).append(im)
    for info in amostra["bags"]:
        pedidas = por_bag.get(info["nome"])
        if not pedidas:
            continue
        bag = encontrar_bag(info)
        if bag is None:
            print(f"Aviso: o bag {info['nome']} não foi encontrado ({info['caminho']}, nem em data/): "
                  f"faltam {len(pedidas)} imagens.")
            continue
        faltam = {(im["camara"], im["stamp_ns"]): im for im in pedidas}
        topicos = list(info["camaras"].values())
        inicio, fim = min(s for _, s in faltam), max(s for _, s in faltam)
        print(f"A ler {len(faltam)} imagens de {bag.name}...")
        for m in leitura_bag.ler_mensagens(bag, topicos, leitura_bag.bag_tem_indice(bag), inicio, fim):
            cam = next((c for c, t in info["camaras"].items() if t == m.channel.topic), None)
            im = faltam.pop((cam, m.log_time_ns), None)
            if im is not None:
                img = leitura_bag.imagem_bgr(m.ros_msg)
                if img is not None:
                    cv2.imwrite(str(destino / f"{im['id']}_original.png"), img)
            if not faltam:
                break


def imagem(amostra, im, corrigir=True):
    """Imagem (BGR) de uma entrada da amostra, corrigida da distorção como no processamento."""
    f = pasta_imagens(amostra["nome"]) / f"{im['id']}_original.png"
    if not f.exists():
        extrair_imagens(amostra)
    img = cv2.imread(str(f))
    if img is None:
        return None
    cal = (amostra.get("calibracao") or {}).get(im["camara"])
    if corrigir and cal:
        img = corretor(cal)(img)
    return img


_CORRETORES = {}


def corretor(cal):
    chave = json.dumps(cal, sort_keys=True)
    if chave not in _CORRETORES:
        _CORRETORES[chave] = leitura_bag.CorretorDistorcao(leitura_bag.calibracao_de_dict(cal))
    return _CORRETORES[chave]


# --------------------------------------------------------------------------- anotações
def ficheiro_anotacao(nome, id_):
    return pasta(nome) / "anotacoes" / f"{id_}.json"


def ler_anotacao(nome, im):
    f = ficheiro_anotacao(nome, im["id"])
    if f.exists():
        return json.loads(f.read_text(encoding="utf-8"))
    return {"id": im["id"], "bag": im["bag"], "camara": im["camara"], "stamp_ns": im["stamp_ns"],
            "objetos": [], "linhas": [], "estrada_contornos": [], "estrada_buracos": [],
            "faixa_ao_lado": None, "ignorar": False}


def gravar_anotacao(nome, a, largura, altura, corrigida):
    a = dict(a, largura=int(largura), altura=int(altura), corrigida=bool(corrigida),
             anotado_em=datetime.datetime.now().isoformat(timespec="seconds"))
    f = ficheiro_anotacao(nome, a["id"])
    f.parent.mkdir(parents=True, exist_ok=True)
    f.write_text(json.dumps(a, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")


def mascara(a, largura, altura):
    """Área circulável marcada (polígonos menos buracos) -> máscara booleana."""
    return registo.mascara_dos_contornos(a.get("estrada_contornos") or [], a.get("estrada_buracos") or [],
                                         largura, altura)


def caixas(a):
    """Objetos marcados -> [(x1, y1, x2, y2, classe, 1.0)], o formato das deteções."""
    return [(*o["caixa"], o["classe"], 1.0) for o in a.get("objetos", [])]


def linhas(a):
    return [np.array(l, dtype=np.float32) for l in a.get("linhas", []) if len(l) > 1]
