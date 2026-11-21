# -*- coding: utf-8 -*-
"""
gui_comum.py - Partes comuns às janelas de resultados.

Usado pelo gui_resultados.py (offline: tab Resumo) e pelo resumo_online.py (resumo no fim do
processamento em tempo real), para os dois calcularem e mostrarem o resumo das deteções da mesma
forma: cores, gamas de veículos, cartões e os três cartões das deteções (linhas e área circulável,
deteções por categoria e veículos).

Só precisa do customtkinter (o mapa, tkintermapview, é só do gui_resultados.py).
"""
import sys
from tkinter import ttk

import registo

try:
    import customtkinter as ctk
except ImportError as e:
    print(f"Erro: falta a biblioteca '{e.name}'. Instale com: pip install customtkinter")
    sys.exit(1)

# --- Gamas do número de veículos (YOLO) e respetivas cores ---
# As gamas (mínimo, máximo, rótulo) estão em registo.GAMAS_VEICULOS; aqui fica a cor de cada uma.
# Veículos = registo.VEICULOS (car, motorcycle, bicycle, bus, truck).
CORES_GAMAS = ["#22C55E", "#FACC15", "#F97316", "#EF4444"]          # verde, amarelo, laranja, vermelho
GAMAS_VEICULOS = [(a, b, r, cor) for (a, b, r), cor in zip(registo.GAMAS_VEICULOS, CORES_GAMAS)]
NOMES_CLASSES = {"car": "carro", "truck": "camião", "bus": "autocarro", "motorcycle": "mota",
                 "bicycle": "bicicleta", "person": "pessoa", "traffic light": "semáforo",
                 "stop sign": "sinal STOP", "train": "comboio"}

# Cores da janela
FUNDO, CARTAO, CARTAO_2 = "#14171C", "#1C2027", "#232832"
TEXTO, TEXTO_2, TEXTO_3 = "#E5E7EB", "#9CA3AF", "#6B7280"
ACENTO = "#3B82F6"


def cor_da_gama(n):
    for a, b, _, cor in GAMAS_VEICULOS:
        if a <= n <= b:
            return cor
    return GAMAS_VEICULOS[-1][3]


def misturar(cor, fundo, f):
    """Mistura cor e fundo ('#RRGGBB'); f = peso da cor (0 a 1)."""
    a = [int(cor[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(fundo[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x * f + y * (1 - f)):02x}" for x, y in zip(a, b))


def tempo_txt(seg):
    if seg is None:
        return "—"
    m, s = divmod(max(0.0, seg), 60)
    return f"{int(m):02d}:{s:04.1f}"


def pct(x):
    return "—" if x is None else f"{x * 100:.0f}%"


def px(widget, valor):
    """Píxeis à escala do ecrã (o customtkinter já escala os seus elementos; os do tk/ttk não)."""
    try:
        esc = ctk.ScalingTracker.get_widget_scaling(widget)
    except Exception:
        esc = 1.0
    return int(round(valor * esc))


def estilo_tabelas(janela):
    estilo = ttk.Style(janela)
    estilo.theme_use("default")
    estilo.configure("Treeview", background=CARTAO_2, fieldbackground=CARTAO_2, foreground=TEXTO,
                     rowheight=px(janela, 26), borderwidth=0, font=("Segoe UI", 10))
    estilo.configure("Treeview.Heading", background=CARTAO, foreground=TEXTO_2, relief="flat",
                     font=("Segoe UI", 10, "bold"))
    estilo.map("Treeview", background=[("selected", "#2D3748")])
    estilo.map("Treeview.Heading", background=[("active", CARTAO)])


def cartao(pai, titulo):
    f = ctk.CTkFrame(pai, fg_color=CARTAO_2, corner_radius=10)
    ctk.CTkLabel(f, text=titulo, font=ctk.CTkFont(size=13, weight="bold"), text_color=TEXTO,
                 anchor="w").pack(fill="x", padx=14, pady=(10, 6))
    return f


def linhas_chave_valor(pai, pares):
    corpo = ctk.CTkFrame(pai, fg_color="transparent")
    corpo.pack(fill="x", padx=14, pady=(0, 12))
    for i, (k, v) in enumerate(pares):
        ctk.CTkLabel(corpo, text=k, text_color=TEXTO_2, font=ctk.CTkFont(size=12), anchor="w").grid(
            row=i, column=0, sticky="w", pady=1)
        ctk.CTkLabel(corpo, text=v, text_color=TEXTO, font=ctk.CTkFont(size=12), anchor="w",
                     justify="left", wraplength=330).grid(row=i, column=1, sticky="w", padx=(12, 0), pady=1)
    return corpo


# --------------------------------------------------------------------------- #
# Resumo das deteções (o mesmo no offline e no online)
# --------------------------------------------------------------------------- #
# O cálculo está no registo.py (para o tempo real o poder fazer sem a janela)
calcular_resumo = registo.resumo_deteccoes


def desenhar_deteccoes(corpo, resumo, modelos):
    """
    Os três cartões das deteções, na grelha de 3 colunas de corpo: «Linhas e área circulável»
    (linha 0, coluna 2), deteções por categoria (linha 1, colunas 0 e 1) e veículos (linha 1, coluna 2).
    resumo: o de calcular_resumo (registo.resumo_deteccoes); modelos: {"objetos": {"nome": ...}, "linhas": ..., "estrada": ...}.
    """
    cams, duas = [str(c) for c in resumo["camaras"]], resumo["duas"]
    est, est_cam, veic = resumo["est"], resumo["est_cam"], resumo["veic"]

    def por_camara(f):
        """'5: x  ·  8: y' (um valor por câmara)."""
        return "  ·  ".join(f"{c}: {f(est_cam[c])}" for c in cams)

    # --- CLRNet e YOLOP (com duas câmaras: um valor por câmara) ---
    c = cartao(corpo, "Linhas e área circulável")
    c.grid(row=0, column=2, sticky="nsew", padx=6, pady=6)
    li, es = est["linhas"], est["estrada"]
    if duas:
        fmt_l = lambda x: "—" if x["linhas"]["media"] is None else f"{x['linhas']['media']:.2f}"
        linhas_chave_valor(c, [
            (modelos.get("linhas", {}).get("nome", "Linhas"), "por câmara"),
            ("  Linhas por imagem", por_camara(fmt_l)),
            ("  Imagens sem linhas", por_camara(lambda x: pct(x["linhas"]["pct_sem_linhas"]))),
            (modelos.get("estrada", {}).get("nome", "Estrada"), "por câmara"),
            ("  Área circulável média", por_camara(lambda x: pct(x["estrada"]["media"]))),
            ("  Mínima / máxima", por_camara(lambda x: f"{pct(x['estrada']['minimo'])} / {pct(x['estrada']['maximo'])}")),
        ])
    else:
        linhas_chave_valor(c, [
            (modelos.get("linhas", {}).get("nome", "Linhas"), ""),
            ("  Linhas por imagem", "—" if li["media"] is None else f"{li['media']:.2f}"),
            ("  Imagens sem linhas", pct(li["pct_sem_linhas"])),
            (modelos.get("estrada", {}).get("nome", "Estrada"), ""),
            ("  Área circulável média", pct(es["media"])),
            ("  Mínima / máxima", f"{pct(es['minimo'])} / {pct(es['maximo'])}"),
        ])
    # --- Objetos por categoria (com duas câmaras: também as deteções de cada uma) ---
    c = cartao(corpo, f"{modelos.get('objetos', {}).get('nome', 'YOLO')}  ·  deteções por categoria"
               + ("  ·  câmaras " + registo.texto_camaras(cams) if duas else ""))
    c.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=6, pady=6)
    colunas = ["classe", "det"] + [f"cam{c_}" for c_ in cams if duas] + ["media", "max", "img", "conf"]
    cab = {"classe": "Categoria", "det": "Deteções", "media": "Média por imagem",
           "max": "Máx. numa imagem", "img": "% imagens", "conf": "Confiança média"}
    cab.update({f"cam{c_}": f"Câmara {c_}" for c_ in cams})
    tabela = ttk.Treeview(c, columns=colunas, show="headings",
                          height=max(3, min(9, len(est["objetos"]["classes"]))))
    for col in colunas:
        tabela.heading(col, text=cab[col])
        tabela.column(col, width=px(corpo, 210 if col == "classe" else (80 if col.startswith("cam") else 100)),
                      anchor="w" if col == "classe" else "center")
    for nome, v in est["objetos"]["classes"].items():
        rotulo = f"{nome} ({NOMES_CLASSES[nome]})" if nome in NOMES_CLASSES else nome
        if nome in registo.VEICULOS:
            rotulo += "  · veículo"
        por_cam = [est_cam[c_]["objetos"]["classes"].get(nome, {}).get("deteccoes", 0) for c_ in cams] if duas else []
        tabela.insert("", "end", values=(rotulo, v["deteccoes"], *por_cam, f"{v['media_por_imagem']:.2f}",
                                         v["maximo"], pct(v["pct_imagens"]), f"{v['confianca_media']:.2f}"))
    tabela.pack(fill="x", padx=14, pady=(0, 6))
    nota = "As deteções são somadas imagem a imagem: o mesmo veículo visto em 30 imagens conta 30 vezes."
    if duas:
        nota += " Média, máximo e % de imagens contam as imagens das duas câmaras."
    ctk.CTkLabel(c, text=nota, text_color=TEXTO_3, font=ctk.CTkFont(size=11), anchor="w", justify="left",
                 wraplength=px(corpo, 760)).pack(fill="x", padx=14, pady=(0, 10))
    # --- Veículos por gama (as cores do mapa do offline) ---
    c = cartao(corpo, "Veículos por instante (soma das câmaras)" if duas else "Veículos por imagem")
    c.grid(row=1, column=2, sticky="nsew", padx=6, pady=6)
    pares = [("Média / máximo", "—" if veic["media"] is None else f"{veic['media']:.2f} / {veic['maximo']}")]
    if duas:
        pares.append(("Por câmara (média)", por_camara(
            lambda x: "—" if x["veiculos"]["media"] is None else f"{x['veiculos']['media']:.2f}")))
    linhas_chave_valor(c, pares)
    for (a, b, rotulo, cor), (_, fr) in zip(GAMAS_VEICULOS, veic["gamas"]):
        lin = ctk.CTkFrame(c, fg_color="transparent")
        lin.pack(fill="x", padx=14, pady=2)
        ctk.CTkFrame(lin, width=10, height=10, corner_radius=5, fg_color=cor).pack(side="left", padx=(0, 6))
        ctk.CTkLabel(lin, text=f"{rotulo} veículos", text_color=TEXTO,
                     width=110, anchor="w", font=ctk.CTkFont(size=12)).pack(side="left")
        barra = ctk.CTkProgressBar(lin, progress_color=cor, fg_color=CARTAO, height=10)
        barra.set(fr)
        barra.pack(side="left", fill="x", expand=True, padx=8)
        ctk.CTkLabel(lin, text=pct(fr), text_color=TEXTO_2, width=40, font=ctk.CTkFont(size=12)).pack(side="left")
    texto = ("% dos instantes em cada gama; em cada instante somam-se as duas câmaras ("
             if duas else "% das imagens em cada gama (") + ", ".join(registo.VEICULOS) + ")"
    ctk.CTkLabel(c, text=texto, text_color=TEXTO_3, font=ctk.CTkFont(size=11), anchor="w", wraplength=360,
                 justify="left").pack(fill="x", padx=14, pady=(4, 12))
