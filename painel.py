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
aviso_faixa.py (zonas de cada lado); aqui junta-se tudo da mesma forma, mais a coluna dos avisos
(mudança de faixa, saída de faixa, colisão e planeador) e o destaque dos objetos com aviso de colisão.
"""
import numpy as np

import aviso_colisao
import aviso_faixa
import aviso_saida_faixa
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
                  titulo=None, colisao=None):
    """Vista de cima (grelha local) com as linhas projetadas, as câmaras, a zona da faixa ao lado de cada
    lado (com a cor do aviso de mudança de faixa) e os objetos seguidos (com o nível do aviso de colisão e
    o tempo até contacto)."""
    import cv2
    vista = grelha.imagem(grelha_atual, altura, linhas=sum((linhas_chao or {}).values(), []),
                          camaras=list((geos or {}).values()), titulo=titulo)
    if avisos and p_aviso is not None:
        aviso_faixa.desenhar(vista, grelha, avisos, p_aviso)
    escala = vista.shape[0] / grelha.largura
    for o in (colisao or {}).get("objetos", []):
        if o.get("x") is None:
            continue
        c = (int(round((grelha.y_max - o["y"]) / grelha.res * escala)),
             int(round((grelha.x_max - o["x"]) / grelha.res * escala)))
        cor = aviso_colisao.COR.get(o.get("nivel"), (230, 230, 230)) if o.get("nivel") != "LIVRE" else (230, 230, 230)
        cv2.circle(vista, c, 4, cor, -1, cv2.LINE_AA)
        if o.get("ttc_s") is not None:
            cv2.putText(vista, f"{o['ttc_s']:.1f}s", (c[0] + 5, c[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, cor, 1,
                        cv2.LINE_AA)
    return vista


def _bgr_para_rgb(c):
    return (c[2], c[1], c[0])


def quadro_avisos(altura, faixa=None, saida=None, colisao=None, planeador=None, largura=300):
    """Coluna com o estado dos avisos (texto com acentos, como o painel das câmaras): mudança de faixa,
    saída de faixa e colisão, de cada lado, e o estado do planeador A* (None = não corre)."""
    import cv2
    from PIL import Image, ImageDraw
    esc = max(0.8, min(1.4, altura / 480))
    base = Image.new("RGB", (int(largura * esc), int(altura)), (24, 27, 32))
    d = ImageDraw.Draw(base)
    f_tit, f_sec = visual.fonte(int(17 * esc), True), visual.fonte(int(13 * esc), True)
    f_txt, f_pq = visual.fonte(int(13 * esc)), visual.fonte(int(11 * esc))
    x0, y = int(12 * esc), int(10 * esc)
    lim = base.size[0] - x0 - int(6 * esc)

    def linha(txt, fonte, cor=(220, 223, 228), recuo=0):
        nonlocal y
        palavras, atual = txt.split(" "), ""
        for pal in palavras:                      # quebra o texto que não cabe na largura
            teste = (atual + " " + pal).strip()
            if d.textlength(teste, font=fonte) > lim - recuo and atual:
                d.text((x0 + recuo, y), atual, font=fonte, fill=cor)
                y += int(fonte.size * 1.3) if hasattr(fonte, "size") else 16
                atual = pal
            else:
                atual = teste
        d.text((x0 + recuo, y), atual, font=fonte, fill=cor)
        y += int(fonte.size * 1.35) if hasattr(fonte, "size") else 16

    def item(nome, txt, cor_bgr):
        r = int(5 * esc)
        d.ellipse([x0, y + r // 2, x0 + 2 * r, y + r // 2 + 2 * r], fill=_bgr_para_rgb(cor_bgr))
        linha(f"{nome}: {txt}", f_txt, recuo=3 * r)

    linha("Avisos", f_tit, (255, 255, 255))
    y += int(4 * esc)
    linha("Mudança de faixa", f_sec, (156, 163, 175))
    for lado in aviso_faixa.LADOS:
        info = (faixa or {}).get(lado)
        if info:
            item(lado.capitalize(), aviso_faixa.texto(info), aviso_faixa.COR.get(info["estado"], (140, 140, 140)))
    if faixa:
        modos = {lado: (faixa.get(lado) or {}).get("faixa", "centro") for lado in aviso_faixa.LADOS}
        linha("faixa ao lado: " + ", ".join(f"{l[:3]}. pelas {m}" if m == "linhas" else f"{l[:3]}. pelo centro"
                                            for l, m in modos.items()), f_pq, (130, 136, 146))
    y += int(6 * esc)
    linha("Saída de faixa", f_sec, (156, 163, 175))
    for lado in aviso_saida_faixa.LADOS:
        info = (saida or {}).get(lado)
        if info:
            item(lado.capitalize(), aviso_saida_faixa.texto(info),
                 (40, 40, 220) if info.get("perigo") else aviso_saida_faixa.COR.get(info["estado"], (140, 140, 140)))
    y += int(6 * esc)
    linha("Colisão", f_sec, (156, 163, 175))
    if colisao:
        item("Estado", aviso_colisao.texto(colisao), aviso_colisao.COR.get(colisao.get("estado"), (140, 140, 140)))
        n = sum(1 for o in colisao.get("objetos", []) if o.get("ttc_s") is not None)
        linha(f"{len(colisao.get('objetos', []))} objetos seguidos, {n} a aproximar-se", f_pq, (130, 136, 146))
    y += int(6 * esc)
    linha("Planeador A*", f_sec, (156, 163, 175))
    linha(planeador if planeador else "não corre (offline)" if planeador is None else "sem estado", f_txt)
    return cv2.cvtColor(np.asarray(base), cv2.COLOR_RGB2BGR)


def desenhar_alertas(img, objetos, cam, stamp_ns):
    """Na imagem de uma câmara (já com o painel), destaca os objetos com aviso de colisão (caixa grossa
    na cor do nível e o tempo até contacto)."""
    import cv2
    for o in objetos or []:
        if str(o.get("camara")) != str(cam) or o.get("stamp_ns") != stamp_ns or o.get("nivel", "LIVRE") == "LIVRE":
            continue
        x1, y1, x2, y2 = (int(round(v)) for v in o["caixa"])
        cor = aviso_colisao.COR.get(o["nivel"], (0, 0, 255))
        cv2.rectangle(img, (x1, y1), (x2, y2), cor, 3)
        txt = f"TTC {o['ttc_s']:.1f} s" if o.get("ttc_s") is not None else "perto"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        yb = min(img.shape[0] - 2, y2 + th + 6)
        cv2.rectangle(img, (x1, yb - th - 6), (x1 + tw + 8, yb + 2), cor, -1)
        cv2.putText(img, txt, (x1 + 4, yb - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img


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
