# -*- coding: utf-8 -*-
"""
visual.py - Desenho dos resultados e do painel de informação no vídeo.

Só trata da apresentação: as deteções (objetos, linhas, estrada) são feitas noutros módulos e
chegam aqui já calculadas. Cores fixas por modelo e, nos objetos, por classe, para a mesma coisa
ter sempre a mesma cor em todas as imagens.

O texto é escrito com o Pillow (instalado com o ultralytics) e uma fonte do sistema, para ter
acentos e um aspeto mais limpo do que as fontes do OpenCV; se não houver nenhuma fonte, usa a
fonte do Pillow.
"""
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def _bgr(hexa):
    """'#RRGGBB' -> (B, G, R), a ordem de cores do OpenCV."""
    hexa = hexa.lstrip("#")
    return (int(hexa[4:6], 16), int(hexa[2:4], 16), int(hexa[0:2], 16))


# --- Cores por modelo (marcador no painel e desenho) ---
COR_MODELO = {
    "objetos": "#3B82F6",   # azul
    "linhas": "#FACC15",    # amarelo
    "estrada": "#22C55E",   # verde
}
COR_DESLIGADO = "#6B7280"   # cinzento

# --- Cores fixas por classe do YOLO (nomes do COCO) ---
# Veículos em azuis/roxos, utilizadores vulneráveis em vermelhos/rosas, sinalização em laranjas.
# Nenhuma usa o amarelo das linhas nem o verde da estrada.
COR_CLASSE = {
    "car": "#3B82F6",
    "truck": "#6366F1",
    "bus": "#A855F7",
    "train": "#7C3AED",
    "motorcycle": "#EC4899",
    "bicycle": "#F472B6",
    "person": "#EF4444",
    "traffic light": "#F97316",
    "stop sign": "#FB923C",
}
# Outras classes (com --classes-objetos todas): cor fixa escolhida pelo nome, desta lista
COR_OUTRAS = ["#0EA5E9", "#14B8A6", "#8B5CF6", "#F43F5E", "#D946EF", "#06B6D4", "#64748B", "#E11D48"]


def cor_classe(nome):
    if nome in COR_CLASSE:
        return COR_CLASSE[nome]
    return COR_OUTRAS[sum(nome.encode()) % len(COR_OUTRAS)]   # sempre a mesma para o mesmo nome


# --- Fontes ---
_FONTES_NORMAL = ["C:/Windows/Fonts/segoeui.ttf", "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
                  "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/System/Library/Fonts/Helvetica.ttc",
                  "/Library/Fonts/Arial.ttf"]
_FONTES_NEGRITO = ["C:/Windows/Fonts/seguisb.ttf", "C:/Windows/Fonts/segoeuib.ttf",
                   "/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf",
                   "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/System/Library/Fonts/Helvetica.ttc",
                   "/Library/Fonts/Arial Bold.ttf"]
_cache_fontes = {}


def fonte(tamanho, negrito=False):
    chave = (tamanho, negrito)
    if chave not in _cache_fontes:
        for caminho in (_FONTES_NEGRITO if negrito else _FONTES_NORMAL):
            if Path(caminho).exists():
                try:
                    _cache_fontes[chave] = ImageFont.truetype(caminho, tamanho)
                    break
                except OSError:
                    pass
        else:
            try:
                _cache_fontes[chave] = ImageFont.load_default(tamanho)
            except TypeError:   # Pillow antigo
                _cache_fontes[chave] = ImageFont.load_default()
    return _cache_fontes[chave]


def _rgba(hexa, alfa=255):
    hexa = hexa.lstrip("#")
    return (int(hexa[0:2], 16), int(hexa[2:4], 16), int(hexa[4:6], 16), alfa)


# --------------------------------------------------------------------------- #
# Desenho das deteções (OpenCV, sobre a imagem BGR)
# --------------------------------------------------------------------------- #
def desenhar_estrada(img, mascara, opacidade=0.30):
    """Área circulável: verde semitransparente, com o contorno a cheio."""
    if mascara is None or not mascara.any():
        return img
    cor = np.array(_bgr(COR_MODELO["estrada"]), dtype=np.float32)
    img[mascara] = (img[mascara] * (1 - opacidade) + cor * opacidade).astype(np.uint8)
    contornos, _ = cv2.findContours(mascara.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos = [c for c in contornos if cv2.contourArea(c) > 200]   # ignora manchas pequenas
    cv2.drawContours(img, contornos, -1, _bgr(COR_MODELO["estrada"]), 2, cv2.LINE_AA)
    return img


def desenhar_linhas(img, linhas, espessura=3):
    """Linhas da estrada: todas na cor do modelo, com um contorno escuro fino para se verem em qualquer fundo."""
    for pts in linhas:
        p = np.round(pts).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [p], False, (20, 20, 20), espessura + 2, cv2.LINE_AA)
        cv2.polylines(img, [p], False, _bgr(COR_MODELO["linhas"]), espessura, cv2.LINE_AA)
    return img


def desenhar_caixas(img, caixas):
    """Objetos: retângulo fino com cantos reforçados, na cor da classe. caixas: [(x1, y1, x2, y2, nome, conf)]."""
    for x1, y1, x2, y2, nome, _ in caixas:
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        cor = _bgr(cor_classe(nome))
        cv2.rectangle(img, (x1, y1), (x2, y2), cor, 1, cv2.LINE_AA)
        c = max(4, min(16, (x2 - x1) // 4, (y2 - y1) // 4))   # tamanho dos cantos
        for (ax, ay), (dx, dy) in (((x1, y1), (1, 1)), ((x2, y1), (-1, 1)), ((x1, y2), (1, -1)), ((x2, y2), (-1, -1))):
            cv2.line(img, (ax, ay), (ax + dx * c, ay), cor, 3, cv2.LINE_AA)
            cv2.line(img, (ax, ay), (ax, ay + dy * c), cor, 3, cv2.LINE_AA)
    return img


# --------------------------------------------------------------------------- #
# Texto e painel (Pillow, numa só passagem no fim)
# --------------------------------------------------------------------------- #
def _fmt_tempo(seg):
    m, s = divmod(max(0.0, seg), 60)
    return f"{int(m):02d}:{s:04.1f}"


def compor(img, caixas, info):
    """
    Escreve as etiquetas dos objetos e o painel de informação. info é um dicionário com:
      tempo, total (s); subtitulo (texto) e cor_subtitulo; modelos: lista de
      (chave, nome do modelo, resultado, ms, ativo, carregado); rodape (texto).
    Devolve a imagem BGR final.
    """
    imagem = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(imagem)
    largura, altura = imagem.size
    esc = max(0.8, min(1.6, largura / 800))          # escala do texto com o tamanho da imagem

    # Painel: posição e tamanho (calculados primeiro, para as etiquetas não ficarem por baixo dele)
    m, p = int(10 * esc), int(12 * esc)               # margem exterior e interior
    lh = int(21 * esc)                                # altura de cada linha de modelo
    modelos = [mo for mo in info["modelos"] if mo[5]]
    f_tempo, f_sub = fonte(int(18 * esc), True), fonte(int(12 * esc))
    f_txt, f_val = fonte(int(13 * esc)), fonte(int(13 * esc), True)
    # largura: 240 px (à escala), ou mais se o subtítulo ou o rodapé não couberem (até 60% da imagem)
    lp_max = int(largura * 0.6)
    subtitulo, rodape = info["subtitulo"], info["rodape"]
    # (a largura usa os textos mais compridos possíveis, para o painel não mudar de tamanho entre imagens)
    modelo_txt = {"objetos": "00 objetos", "linhas": "0 linhas", "estrada": "100% da imagem"}
    linha_modelo = max([88 * esc + max(d.textlength(mo[2], font=f_val),
                                       d.textlength(modelo_txt.get(mo[0], ""), font=f_val))
                        + 12 * esc + d.textlength("000 ms", font=f_sub) for mo in modelos] or [0])
    precisa = max(d.textlength(subtitulo, font=f_sub), d.textlength(rodape, font=f_sub), linha_modelo) + 2 * p + 2
    lp = int(min(lp_max, max(240 * esc, precisa)))
    while subtitulo and d.textlength(subtitulo, font=f_sub) > lp - 2 * p:   # corta com "…" se não couber
        subtitulo = subtitulo[:-2] + "…"
    hp = 2 * p + int(62 * esc) + lh * len(modelos) + int(24 * esc)
    zona_painel = (m, m, m + lp, m + hp)

    # Etiquetas dos objetos ("car 84%", na cor da classe), das mais confiantes para as menos;
    # uma etiqueta que tape outra (ou o painel) não é escrita, para a imagem não ficar ilegível.
    f_etq = fonte(int(12 * esc), negrito=True)
    th = int(16 * esc)
    ocupado = [zona_painel]
    for x1, y1, x2, y2, nome, conf in sorted(caixas, key=lambda c: -c[5]):
        texto = f"{nome} {conf * 100:.0f}%"
        tw = d.textlength(texto, font=f_etq) + 10 * esc
        ex, ey = int(x1), int(y1) - th - 2
        if ey < 0:
            ey = int(y1) + 2
        ret = (ex, ey, ex + tw, ey + th)
        if any(ret[0] < o[2] and o[0] < ret[2] and ret[1] < o[3] and o[1] < ret[3] for o in ocupado):
            continue
        ocupado.append(ret)
        d.rounded_rectangle(ret, radius=int(4 * esc), fill=_rgba(cor_classe(nome))[:3])
        d.text((ex + 5 * esc, ey + th / 2), texto, font=f_etq, fill=(255, 255, 255), anchor="lm")

    # Painel no canto superior esquerdo (fundo escuro semitransparente: desenhado à parte e misturado)
    recorte = imagem.crop(zona_painel).convert("RGBA")
    camada = Image.new("RGBA", recorte.size, (0, 0, 0, 0))
    dp = ImageDraw.Draw(camada)
    dp.rounded_rectangle([0, 0, lp - 1, hp - 1], radius=int(10 * esc), fill=(15, 17, 21, 175))
    x0, y = p, p
    barra_l = lp - 2 * p
    total = info.get("total") or 0
    texto_tempo = _fmt_tempo(info["tempo"]) + (f"  /  {_fmt_tempo(total)}" if total > 0 else "")
    dp.text((x0, y), texto_tempo, font=f_tempo, fill=(255, 255, 255, 255))
    y += int(25 * esc)
    dp.rounded_rectangle([x0, y, x0 + barra_l, y + int(3 * esc)], radius=2, fill=(255, 255, 255, 50))
    if total > 0:
        frac = min(1.0, max(0.0, info["tempo"] / total))
        dp.rounded_rectangle([x0, y, x0 + max(3, int(barra_l * frac)), y + int(3 * esc)], radius=2,
                             fill=(255, 255, 255, 220))
    y += int(9 * esc)
    dp.text((x0, y), subtitulo, font=f_sub, fill=_rgba(info.get("cor_subtitulo", "#9CA3AF")))
    y += int(20 * esc)
    dp.line([x0, y, x0 + barra_l, y], fill=(255, 255, 255, 40), width=1)
    y += int(8 * esc)
    r = int(5 * esc)
    for chave, nome, valor, ms, ativo, _ in modelos:
        cy = y + lh // 2
        if ativo:
            dp.ellipse([x0, cy - r, x0 + 2 * r, cy + r], fill=_rgba(COR_MODELO[chave]))
            dp.text((x0 + 3 * r, cy), nome, font=f_txt, fill=(229, 231, 235, 255), anchor="lm")
            dp.text((x0 + int(88 * esc), cy), valor, font=f_val, fill=(255, 255, 255, 255), anchor="lm")
            dp.text((x0 + barra_l, cy), f"{ms:.0f} ms", font=f_sub, fill=(156, 163, 175, 255), anchor="rm")
        else:
            dp.ellipse([x0, cy - r, x0 + 2 * r, cy + r], outline=_rgba(COR_DESLIGADO), width=max(1, int(1.5 * esc)))
            dp.text((x0 + 3 * r, cy), nome, font=f_txt, fill=_rgba(COR_DESLIGADO), anchor="lm")
            dp.text((x0 + int(88 * esc), cy), "desligado", font=f_sub, fill=_rgba(COR_DESLIGADO), anchor="lm")
        y += lh
    y += int(6 * esc)
    dp.text((x0, y), rodape, font=f_sub, fill=(156, 163, 175, 255))
    imagem.paste(Image.alpha_composite(recorte, camada).convert("RGB"), zona_painel[:2])

    return cv2.cvtColor(np.asarray(imagem), cv2.COLOR_RGB2BGR)


def ecra_fim(img):
    """Escurece a última imagem e escreve 'Fim do vídeo' ao centro."""
    base = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)).convert("RGBA")
    camada = Image.new("RGBA", base.size, (0, 0, 0, 140))
    d = ImageDraw.Draw(camada)
    esc = max(0.8, min(1.6, base.size[0] / 800))
    cx, cy = base.size[0] / 2, base.size[1] / 2
    d.text((cx, cy - 14 * esc), "Fim do vídeo", font=fonte(int(30 * esc), True), fill=(255, 255, 255, 255), anchor="mm")
    d.text((cx, cy + 22 * esc), "Prima Q para fechar", font=fonte(int(15 * esc)), fill=(209, 213, 219, 255), anchor="mm")
    final = Image.alpha_composite(base, camada).convert("RGB")
    return cv2.cvtColor(np.asarray(final), cv2.COLOR_RGB2BGR)
