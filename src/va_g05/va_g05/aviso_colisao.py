# -*- coding: utf-8 -*-
"""
aviso_colisao.py - Aviso de colisão (veículos e peões) só com as câmaras laterais (G5).

Sem medir distâncias nem velocidades com a câmara: o risco vem do TEMPO ATÉ CONTACTO (TTC) de cada
objeto seguido (seguimento.py), que se tira do ritmo a que a caixa do YOLO cresce na imagem (um
objeto que se aproxima fica maior; TTC = 1 / (d ln(tamanho) / dt)). É o princípio dos avisos de
colisão só com câmara. A posição aproximada no chão (geometria, chão plano: a base da caixa) serve
apenas para saber EM QUE FAIXA está o objeto, para não avisar de um carro que se aproxima noutra faixa
mais longe, nem de um que vira.

Zonas (as mesmas faixas do aviso de mudança de faixa: pelas linhas detetadas ou supondo o carro no
centro, aviso_faixa.py):
  atrás      a própria faixa, atrás do carro (um veículo que vem na mesma faixa);
  esquerda   a faixa ao lado, à esquerda (inclui o ângulo morto e o lado do carro);
  direita    a faixa ao lado, à direita;
  fora       outras faixas, passeios, ou sem posição no chão (base acima do horizonte / muito longe).

Nível de cada objeto (e o do aviso, o mais grave):
  PERIGO    TTC <= ttc_perigo_s numa zona de interesse; ou, sem posição, TTC <= ttc_perigo_s com o
            objeto já grande na imagem (altura >= fracao_grande da imagem): está muito perto;
  AVISO     TTC <= ttc_aviso_s numa zona de interesse;
  ATENCAO   um veículo na própria faixa, atrás, mais perto do que x_perto_atras (segue colado);
  LIVRE     nada disto.
Um objeto que se afasta ou mantém a distância (TTC None) não dá AVISO nem PERIGO: com o carro a andar,
os objetos parados atrás afastam-se.

Com outros sensores (LiDAR, radar, odometria) isto melhora-se: distância e velocidade medidas, e a
velocidade do próprio carro para separar objetos parados de objetos em movimento.
"""
from . import aviso_faixa

POR_OMISSAO = {"ttc_aviso_s": 3.0, "ttc_perigo_s": 1.5, "x_perto_atras": -8.0, "x_tras_max": -30.0,
               "fracao_grande": 0.45}
NIVEL = {"LIVRE": 0, "ATENCAO": 1, "AVISO": 2, "PERIGO": 3}
ZONAS = ("atras", "esquerda", "direita")


def parametros(montagem):
    p = dict(POR_OMISSAO)
    p.update((montagem or {}).get("aviso_colisao") or {})
    return p


def zona(o, faixas, carro_tras, p, largura_faixa):
    """Em que zona está o objeto (pela posição aproximada no chão)."""
    x, y = o.get("x"), o.get("y")
    if x is None or x < float(p["x_tras_max"]):
        return "fora"
    for lado in ("esquerda", "direita"):
        if bool(aviso_faixa.na_faixa(faixas.get(lado), lado, x, y)):
            return lado
    # a própria faixa: entre as linhas interiores (ou largura_faixa centrada no carro)
    esq, dir_ = (faixas.get("esquerda") or {}).get("interior"), (faixas.get("direita") or {}).get("interior")
    y_esq = esq[0] + esq[1] * x if esq else largura_faixa / 2
    y_dir = dir_[0] + dir_[1] * x if dir_ else -largura_faixa / 2
    if y_dir <= y <= y_esq and x <= carro_tras:
        return "atras"
    return "fora"


def avaliar(objetos, faixas, p, p_faixa, carro, alturas_img=None):
    """
    objetos: objetos seguidos (seguimento.py), com "x", "y" (posição aproximada no chão, ou None),
    "ttc_s", "camara"; faixas: aviso_faixa (EstimadorFaixas.atualizar); carro: (trás, frente, largura);
    alturas_img: {câmara: altura da imagem em px} (para "objeto grande").
    Devolve {"estado", "zona", "objeto" (o mais grave), "objetos": [com "zona" e "nivel"]}.
    """
    faixas = faixas or aviso_faixa.faixas_centro(p_faixa)
    lista = []
    for o in objetos or []:
        o = dict(o)
        z = zona(o, faixas, carro[0], p, float(p_faixa["largura_faixa"]))
        ttc = o.get("ttc_s")
        nivel = "LIVRE"
        if z != "fora" and ttc is not None and ttc <= float(p["ttc_perigo_s"]):
            nivel = "PERIGO"
        elif z != "fora" and ttc is not None and ttc <= float(p["ttc_aviso_s"]):
            nivel = "AVISO"
        elif o.get("x") is None and ttc is not None and ttc <= float(p["ttc_perigo_s"]):
            h_img = (alturas_img or {}).get(o.get("camara"))
            if h_img and (o["caixa"][3] - o["caixa"][1]) >= float(p["fracao_grande"]) * h_img:
                nivel, z = "PERIGO", "perto"
        elif z == "atras" and o.get("categoria") == "veiculo" and o["x"] >= float(p["x_perto_atras"]):
            nivel = "ATENCAO"
        o.update(zona=z, nivel=nivel)
        lista.append(o)
    pior = max(lista, key=lambda o: (NIVEL[o["nivel"]], -(o.get("ttc_s") or 99.0)), default=None)
    estado = pior["nivel"] if pior else "LIVRE"
    res = {"estado": estado, "objetos": lista,
           "zonas": {z: max((NIVEL[o["nivel"]] for o in lista if o["zona"] == z), default=0) for z in ZONAS}}
    if pior is not None and estado != "LIVRE":
        res["zona"] = pior["zona"]
        res["objeto"] = {k: pior.get(k) for k in ("classe", "camara", "ttc_s", "x", "y", "id")}
    return res


NOME_ZONA = {"atras": "atrás", "esquerda": "à esquerda", "direita": "à direita", "perto": "muito perto",
             "fora": "fora das faixas"}
NOME_ZONA_ASCII = {"atras": "atras", "esquerda": "a esquerda", "direita": "a direita", "perto": "muito perto",
                   "fora": "fora das faixas"}


def texto(info, ascii=False):
    e = info.get("estado", "LIVRE")
    if e == "LIVRE":
        return "LIVRE"
    o = info.get("objeto") or {}
    onde = (NOME_ZONA_ASCII if ascii else NOME_ZONA).get(info.get("zona"), "")
    partes = [f"{o.get('classe', '?')} {onde}".strip()]
    if o.get("ttc_s") is not None:
        partes.append(f"TTC {o['ttc_s']:.1f} s")
    elif e == "ATENCAO":
        partes.append("segue colado" if not ascii else "segue colado")
    nome = {"ATENCAO": "ATENCAO" if ascii else "ATENÇÃO"}.get(e, e)
    return f"{nome} ({', '.join(partes)})"


COR = {"LIVRE": (80, 190, 80), "ATENCAO": (0, 200, 255), "AVISO": (0, 140, 255), "PERIGO": (40, 40, 220)}
