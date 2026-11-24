# -*- coding: utf-8 -*-
"""
painel.py - Desenho comum do ecrã (câmaras com as deteções e o painel, e a vista de cima com a grelha
local e o aviso de mudança de faixa).

É o mesmo em todos os sítios onde os resultados se veem, para o online e o offline terem o mesmo
aspeto:
  visualizador.py      online (Docker), ao vivo e em reprodução: a partir dos tópicos ROS
  processar_bag.py     offline: a partir das deteções de cada imagem, à medida que são feitas
  reproducao.py        offline (reproduzir_registo.py e GUI): a partir do registo das deteções
O desenho em si está no visual.py (deteções e painel), no grelha_local.py (vista de cima) e no
aviso_faixa.py (zonas e estado de cada lado); aqui junta-se tudo da mesma forma.
"""
import numpy as np

import aviso_faixa
import registo
import visual

MODELOS = ("objetos", "linhas", "estrada")
NOME_MODELO = {"objetos": "YOLOv8n", "linhas": "CLRNet", "estrada": "YOLOP"}


def deteccoes_de_json(dados, largura, altura, ativos=None):
    """Deteções no formato do registo (registo.dados_deteccoes) -> (caixas, linhas, máscara da estrada),
    como os detetores as devolvem. A estrada vem dos contornos (None se não houver)."""
    if not dados:
        return [], [], None
    ativos = ativos or dados.get("modelos_ativos") or {k: True for k in MODELOS}
    caixas = [(*o["caixa"], o["classe"], o["confianca"]) for o in dados.get("objetos", [])]
    linhas = [np.array(l, dtype=np.float32) for l in dados.get("linhas", []) if len(l) > 1]
    mascara = None
    if ativos.get("estrada", True) and dados.get("estrada_contornos") is not None:
        mascara = registo.mascara_dos_contornos(dados["estrada_contornos"], dados.get("estrada_buracos", []),
                                                largura, altura)
    return caixas, linhas, mascara


def valores(caixas, linhas, pct_estrada):
    """Texto de cada modelo no painel."""
    n_obj, n_lin = len(caixas), len(linhas)
    return {"objetos": f"{n_obj} objeto" + ("" if n_obj == 1 else "s"),
            "linhas": f"{n_lin} linha" + ("" if n_lin == 1 else "s"),
            "estrada": f"{pct_estrada:.0%} da imagem" if pct_estrada is not None else ""}


def desenhar_deteccoes(frame, caixas, linhas, mascara):
    """Estrada por baixo, linhas por cima, objetos por cima de tudo (no próprio frame)."""
    visual.desenhar_estrada(frame, mascara)
    visual.desenhar_linhas(frame, linhas)
    visual.desenhar_caixas(frame, caixas)
    return frame


def camara(frame, caixas, linhas, mascara, info):
    """Imagem de uma câmara com as deteções e o painel (info: ver visual.compor)."""
    desenhar_deteccoes(frame, caixas, linhas, mascara)
    return visual.compor(frame, caixas, info)


def vista_de_cima(grelha, grelha_atual, altura, linhas_chao=None, geos=None, avisos=None, p_aviso=None,
                  titulo=None):
    """Vista de cima (grelha local) com as linhas projetadas, as câmaras e o aviso de cada lado."""
    vista = grelha.imagem(grelha_atual, altura, linhas=sum((linhas_chao or {}).values(), []),
                          camaras=list((geos or {}).values()), titulo=titulo)
    if avisos and p_aviso is not None:
        aviso_faixa.desenhar(vista, grelha, avisos, p_aviso)
    return vista


def juntar(camaras, imagens, espera=None):
    """Câmaras lado a lado (pela ordem dada); uma câmara sem imagem aparece escura, com o texto de
    espera[c] (ou "à espera da primeira imagem"). Devolve None se ainda não houver nenhuma imagem."""
    ref = next((imagens[c] for c in camaras if imagens.get(c) is not None), None)
    if ref is None:
        return None
    h, w = ref.shape[:2]
    partes = []
    for c in camaras:
        if (espera or {}).get(c):
            partes.append(visual.imagem_espera(w, h, *espera[c]))
        elif imagens.get(c) is None:
            partes.append(visual.imagem_espera(w, h, f"Câmara {c}: à espera da primeira imagem"))
        else:
            partes.append(imagens[c])
    return visual.lado_a_lado(partes)
