# -*- coding: utf-8 -*-
"""
gui_resultados.py - Janela com os resultados dos registos de deteções (feitos pelo processar_bag.py).

Duas tabs:
  1. Resumo   - informação do registo e deteções por modelo e por categoria; filtro por trecho.
  2. Percurso - escolhe-se um registo e um trecho (ou todos); para cada trecho:
                (1) trajeto: coordenada de início e pontos clicados no mapa (a forma do caminho);
                (2) pontos de referência: em instantes fixos do vídeo (a cada 5 s, contados desde o
                    início da gravação) clica-se onde o carro estava;
                (3) mapa, vídeo e tabela de intervalos lado a lado: cada intervalo entre referências é
                    pintado com a cor da média de veículos detetados pelo YOLO nesse intervalo.
                Trajeto e referências podem ser guardados e carregados (ficheiros JSON separados).

Os registos são lidos de data_offline (ou da pasta indicada com --pasta-registo). Se não houver
registos, a tab Percurso fica indisponível.

Registos de duas câmaras: no Resumo escolhe-se «Ambas» ou uma câmara (com «Ambas», as tabelas
mostram também os valores de cada câmara); no Percurso escolhe-se as duas (vídeo lado a lado, 5 à
esquerda e 8 à direita, e cor pela soma dos veículos das duas câmaras) ou só uma.

  python gui_resultados.py
  python gui_resultados.py --pasta-registo outra_pasta

Precisa de: pip install customtkinter tkintermapview  (o mapa precisa de internet: OpenStreetMap).
"""
import argparse
import json
import math
import queue
import sys
import threading
import time
import tkinter as tk
import urllib.request
from pathlib import Path
from tkinter import filedialog, ttk

import cv2
from PIL import Image, ImageTk

sys.path.insert(0, str(Path(__file__).resolve().parent))
import registo
from reproducao import ErroRegisto, Reproducao

try:
    import customtkinter as ctk
    import tkintermapview
except ImportError as e:
    print(f"Erro: falta a biblioteca '{e.name}'. Instale com: pip install customtkinter tkintermapview")
    sys.exit(1)

# --- Gamas do número de veículos (YOLO) e respetivas cores no percurso ---
# (mínimo, máximo, rótulo, cor). Veículos = registo.VEICULOS (car, motorcycle, bicycle, bus, truck).
GAMAS_VEICULOS = [
    (0, 0, "0", "#22C55E"),
    (1, 2, "1–2", "#FACC15"),
    (3, 5, "3–5", "#F97316"),
    (6, 10 ** 9, "6 ou mais", "#EF4444"),
]
INTERVALO_REFERENCIAS_S = 5      # pontos de referência a cada 5 s (contados desde o início da gravação)
COR_PERCURSO_POR_FAZER = "#6B7280"
NOMES_CLASSES = {"car": "carro", "truck": "camião", "bus": "autocarro", "motorcycle": "mota",
                 "bicycle": "bicicleta", "person": "pessoa", "traffic light": "semáforo",
                 "stop sign": "sinal STOP", "train": "comboio"}
URL_TESTE_MAPA = "https://tile.openstreetmap.org/0/0/0.png"

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


def descricao_ensaio(pasta):
    e = registo.ler_ensaio(pasta)
    n = len(e.get("trechos", [])) if e else 0
    cams = [c["camara"] for c in registo.camaras_do_ensaio(e)] if e else []
    txt_cams = f", câmaras {registo.texto_camaras(cams)}" if len(cams) > 1 else ""
    return f"{Path(pasta).parent.name}  ·  {Path(pasta).name}  ({n} trecho{'s' if n != 1 else ''}{txt_cams})"


def texto_imagens(trecho):
    """'360 imagens' ou, com duas câmaras, '360 + 358 imagens (câmaras 5 e 8)'."""
    por_cam = trecho.get("imagens_por_camara") or {}
    if len(por_cam) > 1:
        return (" + ".join(str(n) for n in por_cam.values()) + " imagens (câmaras "
                + registo.texto_camaras(list(por_cam)) + ")")
    return f"{trecho['imagens']} imagens"


def resumo_veiculos(instantes, gamas=GAMAS_VEICULOS):
    """Média, máximo e % de instantes em cada gama, a partir de [(instante, n de veículos), ...]."""
    v = [n for _, n in instantes]
    if not v:
        return {"media": None, "maximo": None, "gamas": [(r, 0.0) for _, _, r, _ in gamas]}
    return {"media": sum(v) / len(v), "maximo": max(v),
            "gamas": [(r, sum(1 for x in v if a <= x <= b) / len(v)) for a, b, r, _ in gamas]}


def internet_para_mapa(timeout=4):
    """True se o servidor do mapa (OpenStreetMap) responde."""
    try:
        pedido = urllib.request.Request(URL_TESTE_MAPA, headers={"User-Agent": "aquisicao-astra-gui"})
        with urllib.request.urlopen(pedido, timeout=timeout) as r:
            return r.status == 200
    except Exception:
        return False


# --------------------------------------------------------------------------- #
# Geometria do percurso desenhado
# --------------------------------------------------------------------------- #
def distancia_m(p, q):
    """Distância (m) entre dois pontos (lat, lon) - fórmula de haversine."""
    r = 6371000.0
    la1, lo1, la2, lo2 = map(math.radians, (p[0], p[1], q[0], q[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


class Percurso:
    """Trajeto desenhado (lista de pontos lat, lon): comprimentos, pontos a uma distância e projeções."""

    def __init__(self, pontos):
        self.pontos = list(pontos)
        self.acum = [0.0]
        for p, q in zip(self.pontos, self.pontos[1:]):
            self.acum.append(self.acum[-1] + distancia_m(p, q))
        self.total = self.acum[-1]
        # coordenadas locais em metros (aproximação plana, suficiente para alguns km)
        lat0 = math.radians(self.pontos[0][0]) if self.pontos else 0.0
        self._kx = 6371000.0 * math.pi / 180 * math.cos(lat0)
        self._ky = 6371000.0 * math.pi / 180

    def ponto(self, d):
        """Ponto à distância d (m) do início."""
        d = min(max(d, 0.0), self.total)
        for i in range(1, len(self.acum)):
            if d <= self.acum[i]:
                seg = self.acum[i] - self.acum[i - 1]
                f = (d - self.acum[i - 1]) / seg if seg > 0 else 0.0
                p, q = self.pontos[i - 1], self.pontos[i]
                return (p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f)
        return self.pontos[-1]

    def entre(self, d0, d1):
        """Pontos do trajeto de d0 a d1 (inclui os dois extremos)."""
        meio = [p for p, a in zip(self.pontos, self.acum) if d0 < a < d1]
        return [self.ponto(d0)] + meio + [self.ponto(d1)]

    def _xy(self, p):
        return ((p[1] - self.pontos[0][1]) * self._kx, (p[0] - self.pontos[0][0]) * self._ky)

    def projetar(self, p):
        """Ponto do trajeto mais perto de p: (distância ao longo do trajeto em m, afastamento em m)."""
        px, py = self._xy(p)
        melhor = (0.0, float("inf"))
        for i in range(1, len(self.pontos)):
            ax, ay = self._xy(self.pontos[i - 1])
            bx, by = self._xy(self.pontos[i])
            vx, vy = bx - ax, by - ay
            comp2 = vx * vx + vy * vy
            f = 0.0 if comp2 == 0 else min(1.0, max(0.0, ((px - ax) * vx + (py - ay) * vy) / comp2))
            qx, qy = ax + f * vx, ay + f * vy
            afast = math.hypot(px - qx, py - qy)
            if afast < melhor[1]:
                melhor = (self.acum[i - 1] + f * (self.acum[i] - self.acum[i - 1]), afast)
        return melhor


def inserir_ponto_de_curva(pontos, p):
    """Insere p no trajeto, no troço onde acrescenta menos comprimento (para seguir uma curva)."""
    if len(pontos) < 2:
        return pontos + [p]
    melhor_i = min(range(1, len(pontos)),
                   key=lambda i: distancia_m(pontos[i - 1], p) + distancia_m(p, pontos[i])
                   - distancia_m(pontos[i - 1], pontos[i]))
    return pontos[:melhor_i] + [p] + pontos[melhor_i:]


# --------------------------------------------------------------------------- #
# Janela
# --------------------------------------------------------------------------- #
class App(ctk.CTk):
    def __init__(self, base, ensaio_inicial=None):
        super().__init__()
        ctk.set_appearance_mode("dark")
        self.base = Path(base)
        self.title("Resultados dos registos")
        self.geometry("1320x800")
        self.minsize(1100, 680)
        self.configure(fg_color=FUNDO)
        self._estilo_tabelas()

        cab = ctk.CTkFrame(self, fg_color="transparent")
        cab.pack(fill="x", padx=18, pady=(14, 0))
        ctk.CTkLabel(cab, text="Resultados dos registos", font=ctk.CTkFont(size=20, weight="bold"),
                     text_color=TEXTO).pack(side="left")
        ctk.CTkLabel(cab, text=f"   {self.base.resolve()}", font=ctk.CTkFont(size=12),
                     text_color=TEXTO_3).pack(side="left", pady=(4, 0))

        self.ensaios = registo.listar_ensaios(self.base) if self.base.exists() else []
        self.tabs = ctk.CTkTabview(self, fg_color=CARTAO, segmented_button_selected_color=ACENTO,
                                   command=self._mudou_tab)
        self.tabs.pack(fill="both", expand=True, padx=14, pady=(6, 14))
        self.tab_resumo = TabResumo(self.tabs.add("Resumo"), self, ensaio_inicial)
        self.tab_percurso = TabPercurso(self.tabs.add("Percurso"), self)
        self.tab_resumo.pack(fill="both", expand=True)
        self.tab_percurso.pack(fill="both", expand=True)
        if not self.ensaios:
            # Sem registos: a tab Percurso não está disponível
            self.tabs._segmented_button._buttons_dict["Percurso"].configure(state="disabled")
        self.tabs.set("Resumo")
        self._tab_atual = "Resumo"
        self.protocol("WM_DELETE_WINDOW", self._fechar)

    def px(self, valor):
        """Píxeis à escala do ecrã (o customtkinter já escala os seus elementos; os do tk/ttk não)."""
        try:
            esc = ctk.ScalingTracker.get_widget_scaling(self)
        except Exception:
            esc = 1.0
        return int(round(valor * esc))

    def _estilo_tabelas(self):
        estilo = ttk.Style(self)
        estilo.theme_use("default")
        estilo.configure("Treeview", background=CARTAO_2, fieldbackground=CARTAO_2, foreground=TEXTO,
                         rowheight=self.px(26), borderwidth=0, font=("Segoe UI", 10))
        estilo.configure("Treeview.Heading", background=CARTAO, foreground=TEXTO_2, relief="flat",
                         font=("Segoe UI", 10, "bold"))
        estilo.map("Treeview", background=[("selected", "#2D3748")])
        estilo.map("Treeview.Heading", background=[("active", CARTAO)])

    def _mudou_tab(self):
        nova = self.tabs.get()
        if self._tab_atual == "Percurso" and nova != "Percurso":
            self.tab_percurso.reiniciar()     # ao sair, tudo volta ao início (trecho, coordenadas, percurso)
        self._tab_atual = nova

    def _fechar(self):
        self.tab_percurso.parar()
        self.destroy()


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
# Tab 1: Resumo
# --------------------------------------------------------------------------- #
class TabResumo(ctk.CTkFrame):
    def __init__(self, pai, app, ensaio_inicial=None):
        super().__init__(pai, fg_color="transparent")
        self.app = app
        if not app.ensaios:
            ctk.CTkLabel(self, text=f"Ainda não há registos em '{app.base}'.\n\nFaça um com o processar_bag.py "
                                    "(tecla R, --trechos ou --registar-tudo) e volte a abrir esta janela.",
                         text_color=TEXTO_2, font=ctk.CTkFont(size=14), justify="center").pack(expand=True)
            return
        barra = ctk.CTkFrame(self, fg_color="transparent")
        barra.pack(fill="x", pady=(4, 8))
        ctk.CTkLabel(barra, text="Registo", text_color=TEXTO_2).pack(side="left", padx=(4, 8))
        self.mapa_ensaios = {descricao_ensaio(p): p for p in app.ensaios}
        inicial = next((d for d, p in self.mapa_ensaios.items()
                        if ensaio_inicial and Path(p).resolve() == Path(ensaio_inicial).resolve()),
                       list(self.mapa_ensaios)[-1])          # por omissão, o mais recente
        self.escolha = ctk.CTkOptionMenu(barra, values=list(self.mapa_ensaios), width=420,
                                         command=lambda _: self._carregar())
        self.escolha.set(inicial)
        self.escolha.pack(side="left")
        ctk.CTkLabel(barra, text="Trecho", text_color=TEXTO_2).pack(side="left", padx=(24, 8))
        self.filtro = ctk.CTkSegmentedButton(barra, values=["Todos"], command=lambda _: self._mostrar())
        self.filtro.pack(side="left")
        # filtro de câmara: só aparece nos registos com duas câmaras
        self.zona_camara = ctk.CTkFrame(barra, fg_color="transparent")
        self.zona_camara.pack(side="left")
        ctk.CTkLabel(self.zona_camara, text="Câmara", text_color=TEXTO_2).pack(side="left", padx=(24, 8))
        self.filtro_camara = ctk.CTkSegmentedButton(self.zona_camara, values=["Ambas"],
                                                    command=lambda _: self._mostrar())
        self.filtro_camara.pack(side="left")
        self.corpo = ctk.CTkScrollableFrame(self, fg_color="transparent")
        self.corpo.pack(fill="both", expand=True)
        self._carregar()

    def _carregar(self):
        self.pasta = Path(self.mapa_ensaios[self.escolha.get()])
        self.ensaio = registo.ler_ensaio(self.pasta)
        self.info_cams = registo.camaras_do_ensaio(self.ensaio)
        self.cams = [c["camara"] for c in self.info_cams]
        self.det = registo.ler_deteccoes(self.pasta / self.ensaio.get("ficheiro_deteccoes", "deteccoes.mcap"))
        valores = ["Todos"] + [str(t["n"]) for t in self.ensaio.get("trechos", [])]
        self.filtro.configure(values=valores)
        self.filtro.set("Todos")
        if len(self.cams) > 1:
            self.filtro_camara.configure(values=["Ambas"] + [f"Câmara {c}" for c in self.cams])
            self.filtro_camara.set("Ambas")
            self.zona_camara.pack(side="left")
        else:
            self.zona_camara.pack_forget()
        self._mostrar()

    def _escolha_camaras(self):
        if len(self.cams) < 2 or self.filtro_camara.get() == "Ambas":
            return list(self.cams)
        return [self.filtro_camara.get().replace("Câmara ", "")]

    def _mostrar(self):
        for w in self.corpo.winfo_children():
            w.destroy()
        e, escolha = self.ensaio, self.filtro.get()
        trechos = e.get("trechos", [])
        if escolha != "Todos":
            trechos = [t for t in trechos if str(t["n"]) == escolha]
        nums = {t["n"] for t in trechos}
        cams = self._escolha_camaras()
        duas = len(cams) > 1
        # deteções das câmaras e trechos escolhidos: {câmara: {instante: dados}}
        sel = {c: {t: d for t, d in self.det.get(c, {}).items() if d.get("trecho") in nums} for c in cams}
        dados = [sel[c][t] for c in cams for t in sorted(sel[c])]
        gamas = [(a, b, r) for a, b, r, _ in GAMAS_VEICULOS]
        est = registo.estatisticas(dados, gamas)
        est_cam = {c: registo.estatisticas(list(sel[c].values())) for c in cams} if duas else {}
        veic = resumo_veiculos(registo.veiculos_por_instante(sel))
        dur = sum((t["fim_ns"] - t["inicio_ns"]) / 1e9 for t in trechos)

        def por_camara(f):
            """'5: x  ·  8: y' (um valor por câmara)."""
            return "  ·  ".join(f"{c}: {f(est_cam[c])}" for c in cams)

        self.corpo.grid_columnconfigure((0, 1, 2), weight=1, uniform="c")
        # --- Vídeo / registo ---
        c = cartao(self.corpo, "Vídeo e registo")
        c.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        bag, proc = e.get("bag", {}), e.get("processamento", {})
        info = {i["camara"]: i for i in self.info_cams}
        if duas:
            txt_cam = registo.texto_camaras(cams) + "  (vídeo: " + ", ".join(
                f"{c_} à {'esquerda' if k == 0 else 'direita'}" for k, c_ in enumerate(cams)) + ")"
            correcao = "  ·  ".join(f"{c_}: " + ("sim" if info[c_].get("calibracao") else "não") for c_ in cams)
            n_img = f"{len(dados)} imagens (" + " + ".join(str(len(sel[c_])) for c_ in cams) + ")"
        else:
            txt_cam = str(cams[0]) if cams[0] is not None else "—"
            if len(self.cams) > 1:
                txt_cam += f"  (do registo com as câmaras {registo.texto_camaras(self.cams)})"
            calib = info[cams[0]].get("calibracao")
            correcao = f"sim ({calib['origem']})" if calib else "não"
            n_img = f"{len(dados)} imagens"
        linhas_chave_valor(c, [
            ("Bag", bag.get("nome", "—")),
            ("Câmaras" if duas else "Câmara", txt_cam),
            ("Duração do bag", tempo_txt(bag.get("duracao_s"))),
            ("Correção", correcao),
            ("Processado em", (e.get("criado_em") or "—").replace("T", " ")),
            ("Trechos", "  ".join(f"{t['n']}: {t.get('inicio_s', 0):.1f}–{t.get('fim_s', 0):.1f} s"
                                  for t in e.get("trechos", []))),
            ("Selecionado", ("todos os trechos" if escolha == "Todos" else f"trecho {escolha}")
             + f"  ·  {n_img}  ·  {tempo_txt(dur)}"),
        ])
        # --- Modelos e processamento ---
        c = cartao(self.corpo, "Modelos")
        c.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
        mod, tempos = e.get("modelos", {}), proc.get("tempo_medio_ms", {})
        pares = []
        for k, limiar in (("objetos", "confianca_minima"), ("linhas", "confianca_minima"), ("estrada", "confianca_minima")):
            m = mod.get(k, {})
            if not m.get("carregado", False):
                pares.append((m.get("nome", k), "desligado"))
                continue
            lim = m.get(limiar)
            txt = f"limiar {lim:g}" if isinstance(lim, (int, float)) else "classe mais provável"
            if k in tempos:
                txt += f"  ·  {tempos[k]:.0f} ms/imagem"
            pares.append((m.get("nome", k), txt))
        classes = mod.get("objetos", {}).get("classes")
        if isinstance(classes, list):
            pares.append(("Classes", ", ".join(classes)))
        if proc.get("imagens_por_segundo"):
            txt = (f"{proc['imagens_por_segundo']:.1f} imagens/s "
                   f"({'com' if proc.get('com_interface') else 'sem'} janela)")
            if len(self.cams) > 1:
                txt += " - somando as duas câmaras"
            pares.append(("Velocidade", txt))
        linhas_chave_valor(c, pares)
        # --- CLRNet e YOLOP (com duas câmaras: um valor por câmara) ---
        c = cartao(self.corpo, "Linhas e área circulável")
        c.grid(row=0, column=2, sticky="nsew", padx=6, pady=6)
        li, es = est["linhas"], est["estrada"]
        if duas:
            fmt_l = lambda x: "—" if x["linhas"]["media"] is None else f"{x['linhas']['media']:.2f}"
            linhas_chave_valor(c, [
                (mod.get("linhas", {}).get("nome", "Linhas"), "por câmara"),
                ("  Linhas por imagem", por_camara(fmt_l)),
                ("  Imagens sem linhas", por_camara(lambda x: pct(x["linhas"]["pct_sem_linhas"]))),
                (mod.get("estrada", {}).get("nome", "Estrada"), "por câmara"),
                ("  Área circulável média", por_camara(lambda x: pct(x["estrada"]["media"]))),
                ("  Mínima / máxima", por_camara(lambda x: f"{pct(x['estrada']['minimo'])} / {pct(x['estrada']['maximo'])}")),
            ])
        else:
            linhas_chave_valor(c, [
                (mod.get("linhas", {}).get("nome", "Linhas"), ""),
                ("  Linhas por imagem", "—" if li["media"] is None else f"{li['media']:.2f}"),
                ("  Imagens sem linhas", pct(li["pct_sem_linhas"])),
                (mod.get("estrada", {}).get("nome", "Estrada"), ""),
                ("  Área circulável média", pct(es["media"])),
                ("  Mínima / máxima", f"{pct(es['minimo'])} / {pct(es['maximo'])}"),
            ])
        # --- Objetos por categoria (com duas câmaras: também as deteções de cada uma) ---
        c = cartao(self.corpo, f"{mod.get('objetos', {}).get('nome', 'YOLO')}  ·  deteções por categoria"
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
            tabela.column(col, width=self.app.px(210 if col == "classe" else (80 if col.startswith("cam") else 100)),
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
                     wraplength=self.app.px(760)).pack(fill="x", padx=14, pady=(0, 10))
        # --- Veículos por gama (as cores do mapa) ---
        c = cartao(self.corpo, "Veículos por instante (soma das câmaras)" if duas else "Veículos por imagem")
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


# --------------------------------------------------------------------------- #
# Tab 2: Percurso
# --------------------------------------------------------------------------- #
class TabPercurso(ctk.CTkFrame):
    """
    Por trecho: (1) trajeto (forma do caminho, com as curvas); (2) pontos de referência (onde o carro
    estava em instantes fixos do vídeo, de INTERVALO em INTERVALO segundos); (3) mapa, vídeo e
    intervalos lado a lado. A cor de cada intervalo entre referências é a da média de veículos.
    Dentro de cada intervalo, a posição avança à velocidade média do trecho a partir da referência;
    no início do intervalo seguinte salta para a referência seguinte. O que fica por pintar entre o
    ponto estimado e a referência seguinte é o desvio (drift) da estimativa, visível no mapa.
    """

    def __init__(self, pai, app):
        super().__init__(pai, fg_color="transparent")
        self.app = app
        self._fio = None
        self._parar_fio = threading.Event()
        self._apos = None
        self.a_correr = False
        self.reiniciar()

    # ---------- comum ----------
    def _limpar(self):
        self.parar()
        for w in self.winfo_children():
            w.destroy()

    def parar(self):
        """Pára a reprodução (fio de leitura e ciclo de atualização)."""
        self.a_correr = False
        self._parar_fio.set()
        if self._apos is not None:
            try:
                self.after_cancel(self._apos)
            except Exception:
                pass
            self._apos = None

    def _botao_secundario(self, pai, texto, comando, **kw):
        return ctk.CTkButton(pai, text=texto, fg_color=CARTAO, hover_color="#2D3748", command=comando, **kw)

    def reiniciar(self):
        """Passo 0: escolher o registo e o trecho (tudo volta ao início)."""
        self._limpar()
        self.rep = None
        self.trajetos = {}            # n do trecho -> [(lat, lon), ...]
        self.refs = {}                # n do trecho -> [{"t_ns", "lat", "lon", "saltado"}, ...]
        if not self.app.ensaios:
            ctk.CTkLabel(self, text=f"Indisponível: não há registos em '{self.app.base}'.",
                         text_color=TEXTO_2, font=ctk.CTkFont(size=14)).pack(expand=True)
            return
        caixa = ctk.CTkFrame(self, fg_color=CARTAO_2, corner_radius=12)
        caixa.place(relx=0.5, rely=0.42, anchor="center")
        ctk.CTkLabel(caixa, text="Percurso de um registo", font=ctk.CTkFont(size=17, weight="bold"),
                     text_color=TEXTO).grid(row=0, column=0, columnspan=3, sticky="w", padx=22, pady=(18, 4))
        ctk.CTkLabel(caixa, text="Escolha o registo (pasta do ensaio) e o trecho. Depois, para cada trecho: "
                                 "desenhe o trajeto no mapa e marque os pontos de referência (onde estava "
                                 "o carro em instantes fixos do vídeo).",
                     text_color=TEXTO_2, wraplength=540, justify="left").grid(
            row=1, column=0, columnspan=3, sticky="w", padx=22, pady=(0, 14))
        self.mapa_ensaios = {descricao_ensaio(p): p for p in self.app.ensaios}
        ctk.CTkLabel(caixa, text="Registo", text_color=TEXTO_2).grid(row=2, column=0, sticky="w", padx=(22, 10))
        self.escolha_ensaio = ctk.CTkOptionMenu(caixa, values=list(self.mapa_ensaios), width=420,
                                                command=lambda _: self._atualizar_trechos())
        self.escolha_ensaio.set(list(self.mapa_ensaios)[-1])
        self.escolha_ensaio.grid(row=2, column=1, sticky="w")
        self._botao_secundario(caixa, "Outra pasta…", self._outra_pasta, width=110).grid(row=2, column=2, padx=(8, 22))
        ctk.CTkLabel(caixa, text="Trecho", text_color=TEXTO_2).grid(row=3, column=0, sticky="w", padx=(22, 10), pady=10)
        self.escolha_trecho = ctk.CTkOptionMenu(caixa, values=["—"], width=420)
        self.escolha_trecho.grid(row=3, column=1, sticky="w", pady=10)
        # câmaras: só nos registos com duas (as duas lado a lado, ou só uma)
        self.rotulo_camaras = ctk.CTkLabel(caixa, text="Câmaras", text_color=TEXTO_2)
        self.rotulo_camaras.grid(row=4, column=0, sticky="w", padx=(22, 10), pady=(0, 10))
        self.escolha_camaras = ctk.CTkOptionMenu(caixa, values=["—"], width=420)
        self.escolha_camaras.grid(row=4, column=1, sticky="w", pady=(0, 10))
        ctk.CTkLabel(caixa, text="Referências a cada", text_color=TEXTO_2).grid(row=5, column=0, sticky="w", padx=(22, 10))
        linha = ctk.CTkFrame(caixa, fg_color="transparent")
        linha.grid(row=5, column=1, sticky="w")
        self.entrada_intervalo = ctk.CTkEntry(linha, width=60)
        self.entrada_intervalo.insert(0, f"{INTERVALO_REFERENCIAS_S:g}")
        self.entrada_intervalo.pack(side="left")
        ctk.CTkLabel(linha, text="segundos (contados desde o início da gravação)", text_color=TEXTO_3).pack(side="left", padx=8)
        self.msg = ctk.CTkLabel(caixa, text="", text_color="#F59E0B", wraplength=540, justify="left")
        self.msg.grid(row=6, column=0, columnspan=3, sticky="w", padx=22, pady=(8, 0))
        ctk.CTkButton(caixa, text="Continuar", fg_color=ACENTO, command=self._continuar).grid(
            row=7, column=1, columnspan=2, sticky="e", padx=22, pady=(8, 18))
        self._atualizar_trechos()

    def _outra_pasta(self):
        pasta = filedialog.askdirectory(title="Pasta do ensaio (com ensaio.json)", initialdir=str(self.app.base))
        if not pasta:
            return
        if registo.ler_ensaio(pasta) is None:
            self.msg.configure(text="Essa pasta não tem ensaio.json (não é um registo do processar_bag.py).")
            return
        desc = descricao_ensaio(pasta)
        self.mapa_ensaios[desc] = Path(pasta)
        self.escolha_ensaio.configure(values=list(self.mapa_ensaios))
        self.escolha_ensaio.set(desc)
        self._atualizar_trechos()

    def _atualizar_trechos(self):
        e = registo.ler_ensaio(self.mapa_ensaios[self.escolha_ensaio.get()])
        self.opcoes_trecho = {"Todos os trechos": [t["n"] for t in e.get("trechos", [])]}
        for t in e.get("trechos", []):
            self.opcoes_trecho[f"Trecho {t['n']}  ({t.get('inicio_s', 0):.1f}–{t.get('fim_s', 0):.1f} s, "
                               f"{texto_imagens(t)})"] = [t["n"]]
        self.escolha_trecho.configure(values=list(self.opcoes_trecho))
        self.escolha_trecho.set("Todos os trechos" if len(self.opcoes_trecho) > 2 else list(self.opcoes_trecho)[-1])
        cams = [c["camara"] for c in registo.camaras_do_ensaio(e)]
        if len(cams) > 1:
            self.opcoes_camaras = {f"{registo.texto_camaras(cams)}  (vídeo lado a lado, cor pela soma dos veículos)": cams}
            for c in cams:
                self.opcoes_camaras[f"Só a câmara {c}"] = [c]
            self.escolha_camaras.configure(values=list(self.opcoes_camaras))
            self.escolha_camaras.set(list(self.opcoes_camaras)[0])
            self.rotulo_camaras.grid()
            self.escolha_camaras.grid()
        else:
            self.opcoes_camaras = {"": None}
            self.escolha_camaras.set("")
            self.rotulo_camaras.grid_remove()
            self.escolha_camaras.grid_remove()
        self.msg.configure(text="")

    def _continuar(self):
        try:
            self.intervalo_s = float(self.entrada_intervalo.get().replace(",", "."))
            if self.intervalo_s <= 0:
                raise ValueError
        except ValueError:
            self.msg.configure(text="O intervalo entre referências tem de ser um número de segundos maior do que 0.")
            return
        self.msg.configure(text="A carregar o registo…")
        self.update_idletasks()
        try:
            self.rep = Reproducao(self.mapa_ensaios[self.escolha_ensaio.get()],
                                  camaras=self.opcoes_camaras.get(self.escolha_camaras.get()))
        except ErroRegisto as e:
            self.msg.configure(text=str(e))
            return
        self.ordem = self.opcoes_trecho[self.escolha_trecho.get()]
        self._passo_trajeto(0)

    # ---------- mapa ----------
    def _area_mapa(self, pai, ao_clicar=None, tentar_de_novo=None):
        """Cria o mapa; se não houver internet, mostra o aviso (e um botão para tentar de novo)."""
        if not internet_para_mapa():
            aviso = ctk.CTkFrame(pai, fg_color=CARTAO_2, corner_radius=10)
            ctk.CTkLabel(aviso, text="Sem ligação à internet", font=ctk.CTkFont(size=16, weight="bold"),
                         text_color="#F59E0B").pack(pady=(40, 8))
            ctk.CTkLabel(aviso, text="O mapa (OpenStreetMap) é descarregado da internet e não pode ser mostrado.\n"
                                     "Ligue-se à internet e carregue em «Tentar de novo».",
                         text_color=TEXTO_2, justify="center").pack(padx=20)
            ctk.CTkButton(aviso, text="Tentar de novo", fg_color=ACENTO, command=tentar_de_novo).pack(pady=18)
            return aviso, None
        mapa = tkintermapview.TkinterMapView(pai, corner_radius=10, bg_color=CARTAO)
        if ao_clicar:
            mapa.add_left_click_map_command(ao_clicar)
        return mapa, mapa

    def _enquadrar(self, pontos, margem=0.0004):
        if self.mapa is None or not pontos:
            return
        lats, lons = [p[0] for p in pontos], [p[1] for p in pontos]
        if len(pontos) == 1:
            self.mapa.set_position(lats[0], lons[0])
            self.mapa.set_zoom(17)
            return
        caixa = ((max(lats) + margem, min(lons) - margem), (min(lats) - margem, max(lons) + margem))
        self.after(150, lambda: self.mapa is not None and self.mapa.fit_bounding_box(*caixa))

    def _icone_referencia(self, cor="#FFFFFF"):
        """Pequeno círculo para marcar as referências no mapa (não é um marcador de posição)."""
        if not hasattr(self, "_icones"):
            self._icones = {}
        if cor not in self._icones:
            t = self.app.px(14)
            img = Image.new("RGBA", (t, t), (0, 0, 0, 0))
            from PIL import ImageDraw
            d = ImageDraw.Draw(img)
            d.ellipse([1, 1, t - 2, t - 2], fill=cor, outline="#111827", width=max(2, self.app.px(2)))
            self._icones[cor] = ImageTk.PhotoImage(img)
        return self._icones[cor]

    def _titulo_trecho(self, pai, i, subtitulo):
        n = self.ordem[i]
        t = self.rep.trecho(n)
        ctk.CTkLabel(pai, text=f"Trecho {n}" + (f"  ({i + 1} de {len(self.ordem)})" if len(self.ordem) > 1 else ""),
                     font=ctk.CTkFont(size=17, weight="bold"), text_color=TEXTO, anchor="w").pack(fill="x", padx=18, pady=(16, 0))
        cams = (f"  ·  câmaras {registo.texto_camaras(self.rep.camaras)}" if self.rep.duas
                else (f"  ·  câmara {self.rep.camaras[0]}" if len(self.rep.camaras_registo) > 1 else ""))
        ctk.CTkLabel(pai, text=f"{subtitulo}  ·  {t.get('inicio_s', 0):.1f} s a {t.get('fim_s', 0):.1f} s do vídeo{cams}",
                     text_color=TEXTO_2, anchor="w", wraplength=self.app.px(520 if self.rep.duas else 400),
                     justify="left").pack(fill="x", padx=18, pady=(2, 12))

    # ---------- ficheiros (trajeto e referências, em separado) ----------
    def _guardar_json(self, tipo, n, conteudo):
        nome = f"{tipo}_trecho{n}.json"
        caminho = filedialog.asksaveasfilename(title=f"Guardar {tipo}", initialdir=str(self.rep.pasta),
                                               initialfile=nome, defaultextension=".json",
                                               filetypes=[("JSON", "*.json")])
        if not caminho:
            return None
        dados = {"tipo": tipo, "ensaio": self.rep.pasta.name, "trecho": n,
                 "criado_em": time.strftime("%Y-%m-%dT%H:%M:%S"), **conteudo}
        Path(caminho).write_text(json.dumps(dados, indent=2, ensure_ascii=False), encoding="utf-8")
        return caminho

    def _carregar_json(self, tipo, n):
        caminho = filedialog.askopenfilename(title=f"Carregar {tipo}", initialdir=str(self.rep.pasta),
                                             filetypes=[("JSON", "*.json")])
        if not caminho:
            return None, None
        try:
            dados = json.loads(Path(caminho).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None, "Não foi possível ler o ficheiro (não é um JSON válido)."
        if dados.get("tipo") != tipo:
            return None, f"Este ficheiro não é de {tipo} (é de «{dados.get('tipo', '?')}»)."
        aviso = None
        if dados.get("ensaio") != self.rep.pasta.name or dados.get("trecho") != n:
            aviso = (f"Atenção: o ficheiro foi guardado para o trecho {dados.get('trecho')} do registo "
                     f"«{dados.get('ensaio')}».")
        return dados, aviso

    # ---------- passo 1: trajeto ----------
    def _passo_trajeto(self, i):
        self._limpar()
        self.i_trecho = i
        n = self.ordem[i]
        self.pontos = list(self.trajetos.get(n, []))
        self.linha_desenho = None

        esq = ctk.CTkFrame(self, fg_color="transparent")
        esq.pack(side="left", fill="both", expand=True, padx=(0, 8))
        widget, self.mapa = self._area_mapa(esq, self._clique_trajeto, lambda: self._passo_trajeto(i))
        widget.pack(fill="both", expand=True)

        # painel com barra de deslocamento: em ecrãs mais pequenos (ou com zoom), os botões continuam acessíveis
        dir_ = ctk.CTkScrollableFrame(self, fg_color=CARTAO_2, corner_radius=10, width=350)
        dir_.pack(side="right", fill="y")
        self._titulo_trecho(dir_, i, "1. Trajeto")
        ctk.CTkLabel(dir_, text="Coordenadas de início (latitude, longitude)", text_color=TEXTO,
                     anchor="w").pack(fill="x", padx=18)
        linha = ctk.CTkFrame(dir_, fg_color="transparent")
        linha.pack(fill="x", padx=18, pady=(4, 2))
        self.entrada = ctk.CTkEntry(linha, placeholder_text="ex.: 40.6405, -8.6538")
        self.entrada.pack(side="left", fill="x", expand=True)
        self.entrada.bind("<Return>", lambda _: self._ir())
        ctk.CTkButton(linha, text="Ir", width=50, fg_color=ACENTO, command=self._ir).pack(side="left", padx=(6, 0))
        self.msg_passo = ctk.CTkLabel(dir_, text="", text_color="#F59E0B", anchor="w", wraplength=310, justify="left")
        self.msg_passo.pack(fill="x", padx=18)
        ctk.CTkLabel(dir_, text="Desenhe o trajeto clicando no mapa ponto a ponto, pela ordem em que foi feito, "
                                "com pontos suficientes para seguir as curvas das ruas (o primeiro ponto é a "
                                "coordenada de início). Arraste para mover o mapa e use a roda do rato para aproximar.",
                     text_color=TEXTO_2, anchor="w", wraplength=310, justify="left").pack(fill="x", padx=18, pady=(12, 6))
        botoes = ctk.CTkFrame(dir_, fg_color="transparent")
        botoes.pack(fill="x", padx=18)
        self._botao_secundario(botoes, "Desfazer ponto", self._desfazer_trajeto).pack(side="left", expand=True, fill="x", padx=(0, 4))
        self._botao_secundario(botoes, "Limpar", self._limpar_trajeto).pack(side="left", expand=True, fill="x", padx=(4, 0))
        self.info_pontos = ctk.CTkLabel(dir_, text="", text_color=TEXTO_3, anchor="w")
        self.info_pontos.pack(fill="x", padx=18, pady=8)
        ficheiros = ctk.CTkFrame(dir_, fg_color="transparent")
        ficheiros.pack(fill="x", padx=18, pady=(0, 6))
        self._botao_secundario(ficheiros, "Guardar trajeto…", self._guardar_trajeto).pack(side="left", expand=True, fill="x", padx=(0, 4))
        self._botao_secundario(ficheiros, "Carregar trajeto…", self._carregar_trajeto).pack(side="left", expand=True, fill="x", padx=(4, 0))
        self.bt_seguinte = ctk.CTkButton(dir_, text="Seguinte: pontos de referência", fg_color=ACENTO,
                                         state="disabled", command=self._fim_trajeto)
        self.bt_seguinte.pack(fill="x", padx=18, pady=(10, 6))
        ctk.CTkButton(dir_, text="Recomeçar", fg_color="transparent", border_width=1, border_color=TEXTO_3,
                      text_color=TEXTO_2, hover_color=CARTAO, command=self.reiniciar).pack(fill="x", padx=18, pady=(6, 16))
        if self.mapa is None:
            self.entrada.configure(state="disabled")
        elif self.pontos:
            self._enquadrar(self.pontos)
        self._atualizar_trajeto()

    def _ir(self):
        texto = self.entrada.get().replace(";", ",").strip()
        try:
            lat, lon = [float(x) for x in texto.replace(" ", ",").split(",") if x.strip()]
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError
        except ValueError:
            self.msg_passo.configure(text="Coordenadas inválidas. Use «latitude, longitude», ex.: 40.6405, -8.6538")
            return
        self.msg_passo.configure(text="")
        self.mapa.set_position(lat, lon)
        self.mapa.set_zoom(17)
        self.pontos = [(lat, lon)]
        self._atualizar_trajeto()

    def _clique_trajeto(self, coords):
        if not self.pontos:
            self.msg_passo.configure(text="Indique primeiro as coordenadas de início e carregue em «Ir».")
            return
        self.pontos.append(tuple(coords))
        self._atualizar_trajeto()

    def _desfazer_trajeto(self):
        if len(self.pontos) > 1:
            self.pontos.pop()
            self._atualizar_trajeto()

    def _limpar_trajeto(self):
        self.pontos = self.pontos[:1]
        self._atualizar_trajeto()

    def _atualizar_trajeto(self):
        if self.mapa is None:
            return
        if self.linha_desenho is not None:
            self.linha_desenho.delete()
            self.linha_desenho = None
        if len(self.pontos) >= 2:
            self.linha_desenho = self.mapa.set_path(self.pontos, color=ACENTO, width=5)
        n = len(self.pontos)
        comp = Percurso(self.pontos).total if n >= 2 else 0
        self.info_pontos.configure(text="Nenhum ponto." if n == 0 else f"{n} ponto{'s' if n != 1 else ''}  ·  {comp:.0f} m")
        self.bt_seguinte.configure(state="normal" if n >= 2 else "disabled")

    def _guardar_trajeto(self):
        if len(self.pontos) < 2:
            self.msg_passo.configure(text="O trajeto precisa de pelo menos 2 pontos para ser guardado.")
            return
        caminho = self._guardar_json("trajeto", self.ordem[self.i_trecho], {"pontos": [list(p) for p in self.pontos]})
        if caminho:
            self.msg_passo.configure(text=f"Trajeto guardado em {caminho}", text_color=TEXTO_2)

    def _carregar_trajeto(self):
        dados, aviso = self._carregar_json("trajeto", self.ordem[self.i_trecho])
        if dados is None:
            if aviso:
                self.msg_passo.configure(text=aviso, text_color="#F59E0B")
            return
        self.pontos = [tuple(p) for p in dados.get("pontos", [])]
        self.msg_passo.configure(text=aviso or f"Trajeto carregado ({len(self.pontos)} pontos).",
                                 text_color="#F59E0B" if aviso else TEXTO_2)
        self._enquadrar(self.pontos)
        self._atualizar_trajeto()

    def _fim_trajeto(self):
        self.trajetos[self.ordem[self.i_trecho]] = list(self.pontos)
        self._passo_referencias(self.i_trecho)

    # ---------- passo 2: pontos de referência ----------
    def _grelha(self, trecho):
        """Instantes de referência: múltiplos do intervalo (contados desde o início do bag) dentro do trecho."""
        passo = int(round(self.intervalo_s * 1e9))
        base = self.rep.inicio_bag_ns
        k = (trecho["inicio_ns"] - base) // passo + 1
        instantes = []
        margem = 100_000_000                         # 0,1 s: um instante colado ao início/fim não acrescenta nada
        while base + k * passo < trecho["fim_ns"] - margem:
            if base + k * passo > trecho["inicio_ns"] + margem:
                instantes.append(base + k * passo)
            k += 1
        return instantes

    def _passo_referencias(self, i):
        self._limpar()
        self.i_trecho = i
        n = self.ordem[i]
        self.trecho = self.rep.trecho(n)
        self.percurso = Percurso(self.trajetos[n])
        self.grelha = self._grelha(self.trecho)
        anteriores = {r["t_ns"]: r for r in self.refs.get(n, [])}
        self.lista_refs = [anteriores.get(t) for t in self.grelha]    # None = ainda por marcar
        self.j = next((k for k, r in enumerate(self.lista_refs) if r is None), len(self.grelha))
        self.imagens_ref = {}
        self.marcas = []

        esq = ctk.CTkFrame(self, fg_color="transparent")
        esq.pack(side="left", fill="both", expand=True, padx=(0, 8))
        widget, self.mapa = self._area_mapa(esq, self._clique_referencia, lambda: self._passo_referencias(i))
        widget.pack(fill="both", expand=True)
        if self.mapa is not None:
            self.mapa.add_right_click_menu_command("Acrescentar ponto de curva ao trajeto aqui",
                                                   self._ponto_de_curva, pass_coords=True)

        # painel com barra de deslocamento: em ecrãs mais pequenos (ou com zoom), os botões continuam acessíveis
        # com duas câmaras, as imagens ficam lado a lado: painel mais largo e imagem mais baixa
        larg_img, alt_img = (560, 214) if self.rep.duas else (400, 240)
        texto_larg = larg_img
        dir_ = ctk.CTkScrollableFrame(self, fg_color=CARTAO_2, corner_radius=10, width=larg_img + 40)
        dir_.pack(side="right", fill="y")
        self._titulo_trecho(dir_, i, f"2. Pontos de referência (a cada {self.intervalo_s:g} s)")
        moldura = ctk.CTkFrame(dir_, width=larg_img, height=alt_img, fg_color="#000000", corner_radius=6)
        moldura.pack(padx=18)
        moldura.pack_propagate(False)
        self.ecra_ref = tk.Label(moldura, bg="#000000", bd=0)
        self.ecra_ref.pack(fill="both", expand=True)
        self.moldura_ref = moldura
        self.texto_instante = ctk.CTkLabel(dir_, text="", text_color=TEXTO, font=ctk.CTkFont(size=13, weight="bold"),
                                           anchor="w")
        self.texto_instante.pack(fill="x", padx=18, pady=(8, 0))
        self.modo_clique = ctk.CTkSegmentedButton(dir_, values=["Marcar referência", "Acrescentar curva"],
                                                  selected_color=ACENTO, command=lambda _: self._explicar_modo())
        self.modo_clique.set("Marcar referência")
        self.modo_clique.pack(fill="x", padx=18, pady=(6, 2))
        self.explicacao = ctk.CTkLabel(dir_, text="", text_color=TEXTO_2, anchor="w", wraplength=texto_larg, justify="left")
        self.explicacao.pack(fill="x", padx=18, pady=(2, 6))
        botoes = ctk.CTkFrame(dir_, fg_color="transparent")
        botoes.pack(fill="x", padx=18)
        self._botao_secundario(botoes, "Desfazer", self._desfazer_referencia).pack(side="left", expand=True, fill="x", padx=(0, 4))
        self._botao_secundario(botoes, "Saltar", self._saltar_referencia).pack(side="left", expand=True, fill="x", padx=(4, 0))
        self.msg_passo = ctk.CTkLabel(dir_, text="", text_color="#F59E0B", anchor="w", wraplength=texto_larg, justify="left")
        self.msg_passo.pack(fill="x", padx=18, pady=(4, 0))
        self.tabela_refs = ttk.Treeview(dir_, columns=("t", "coord", "dist"), show="headings", height=4)
        for col, txt, larg in (("t", "Instante", 80), ("coord", "Referência (lat, lon)", 190), ("dist", "No trajeto", 90)):
            self.tabela_refs.heading(col, text=txt)
            self.tabela_refs.column(col, width=self.app.px(larg), anchor="center")
        self.tabela_refs.pack(fill="x", padx=18, pady=(6, 4))
        ficheiros = ctk.CTkFrame(dir_, fg_color="transparent")
        ficheiros.pack(fill="x", padx=18, pady=(2, 4))
        self._botao_secundario(ficheiros, "Guardar referências…", self._guardar_referencias).pack(side="left", expand=True, fill="x", padx=(0, 4))
        self._botao_secundario(ficheiros, "Carregar referências…", self._carregar_referencias).pack(side="left", expand=True, fill="x", padx=(4, 0))
        fim = ctk.CTkFrame(dir_, fg_color="transparent")
        fim.pack(fill="x", padx=18, pady=(6, 14))
        self._botao_secundario(fim, "Voltar ao trajeto", lambda: self._passo_trajeto(i)).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ultimo = i == len(self.ordem) - 1
        self.bt_seguinte = ctk.CTkButton(fim, text="Ver percurso e vídeo" if ultimo else "Trecho seguinte",
                                         fg_color=ACENTO, state="disabled", command=self._fim_referencias)
        self.bt_seguinte.pack(side="left", expand=True, fill="x", padx=(4, 0))

        self._explicar_modo()
        self._desenhar_mapa_referencias()
        self._enquadrar(self.percurso.pontos)
        self._atualizar_referencias()
        # imagens dos instantes de referência (lidas do bag num fio à parte)
        self._parar_fio = threading.Event()
        threading.Thread(target=self._ler_imagens_referencia, args=(self.trecho, self.grelha, self._parar_fio),
                         daemon=True).start()
        self._esperar_imagem()

    def _ler_imagens_referencia(self, trecho, grelha, parar):
        pendentes = list(grelha)
        try:
            for t_ns, img, _ in self.rep.imagens(trecho, desenhar=False):
                if "primeira" not in self.imagens_ref:
                    self.imagens_ref["primeira"] = (t_ns, img)
                if parar.is_set() or not pendentes:
                    return
                while pendentes and t_ns >= pendentes[0]:
                    self.imagens_ref[pendentes.pop(0)] = (t_ns, img)
        except Exception as e:
            self.imagens_ref["erro"] = str(e)

    def _esperar_imagem(self):
        """Mostra a imagem do instante atual quando estiver lida."""
        if not hasattr(self, "ecra_ref") or not self.ecra_ref.winfo_exists():
            return
        if "erro" in self.imagens_ref:
            self.texto_instante.configure(text=f"Erro a ler o bag: {self.imagens_ref['erro']}")
            return
        # instante atual; sem instantes por marcar, mostra a primeira imagem do trecho
        chave = self.grelha[self.j] if self.j < len(self.grelha) else "primeira"
        if chave in self.imagens_ref:
            self.moldura_ref.update_idletasks()
            w = max(200, self.moldura_ref.winfo_width())
            h = max(150, self.moldura_ref.winfo_height())
            self._mostrar_em(self.ecra_ref, self.imagens_ref[chave][1], w, h)
        else:
            self._apos = self.after(150, self._esperar_imagem)

    def _mostrar_em(self, rotulo, imagem_bgr, w, h):
        ih, iw = imagem_bgr.shape[:2]
        esc = min(w / iw, h / ih)
        img = cv2.resize(imagem_bgr, (max(1, int(iw * esc)), max(1, int(ih * esc))), interpolation=cv2.INTER_AREA)
        foto = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
        rotulo.configure(image=foto)
        rotulo.image = foto

    def _t_rel(self, t_ns):
        return (t_ns - self.rep.inicio_bag_ns) / 1e9

    def _explicar_modo(self):
        if self.modo_clique.get() == "Acrescentar curva":
            self.explicacao.configure(text="Clique no mapa por onde a rua passa: o ponto é acrescentado ao trajeto "
                                           "no troço mais perto, para seguir a curva. Volte a «Marcar referência» "
                                           "para continuar.")
        else:
            self.explicacao.configure(text="Clique no mapa onde o carro está nesta imagem"
                                           + (" (câmaras " + " | ".join(self.rep.camaras) + ", lado a lado)"
                                              if self.rep.duas else "")
                                           + "; o ponto é encostado ao "
                                           "trajeto. Se não reconhecer o sítio, use «Saltar». (Botão direito no mapa "
                                           "também acrescenta um ponto de curva.)")

    def _clique_referencia(self, coords):
        if self.modo_clique.get() == "Acrescentar curva":
            self._ponto_de_curva(coords)
            return
        if self.j >= len(self.grelha):
            self.msg_passo.configure(text="Todos os instantes já têm referência. Use «Desfazer» para corrigir a última.")
            return
        d, afast = self.percurso.projetar(tuple(coords))
        anterior = max((r["d"] for r in self.lista_refs[:self.j] if r and not r["saltado"]), default=0.0)
        if d < anterior:
            self.msg_passo.configure(text="Esse ponto fica antes da referência anterior no trajeto. "
                                          "Clique mais à frente (ou corrija com «Desfazer»).", text_color="#F59E0B")
            return
        self.msg_passo.configure(text="" if afast < 40 else
                                 f"Aviso: o clique ficou a {afast:.0f} m do trajeto; foi encostado ao ponto mais perto.",
                                 text_color="#F59E0B")
        self.lista_refs[self.j] = {"t_ns": self.grelha[self.j], "lat": coords[0], "lon": coords[1],
                                   "d": d, "saltado": False}
        self._avancar_referencia()

    def _saltar_referencia(self):
        if self.j < len(self.grelha):
            self.lista_refs[self.j] = {"t_ns": self.grelha[self.j], "lat": None, "lon": None, "d": None, "saltado": True}
            self._avancar_referencia()

    def _avancar_referencia(self):
        self.j += 1
        self._desenhar_mapa_referencias()
        self._atualizar_referencias()
        self._esperar_imagem()

    def _desfazer_referencia(self):
        if self.j > 0:
            self.j -= 1
            self.lista_refs[self.j] = None
            self.msg_passo.configure(text="")
            self._desenhar_mapa_referencias()
            self._atualizar_referencias()
            self._esperar_imagem()

    def _ponto_de_curva(self, coords):
        """Botão direito: acrescenta um ponto de curva ao trajeto e reencosta as referências."""
        n = self.ordem[self.i_trecho]
        self.trajetos[n] = inserir_ponto_de_curva(self.trajetos[n], tuple(coords))
        self.msg_passo.configure(text=f"Ponto de curva acrescentado (trajeto com {len(self.trajetos[n])} pontos).",
                                 text_color=TEXTO_2)
        self.percurso = Percurso(self.trajetos[n])
        for r in self.lista_refs:
            if r and not r["saltado"]:
                r["d"] = self.percurso.projetar((r["lat"], r["lon"]))[0]
        self._desenhar_mapa_referencias()
        self._atualizar_referencias()

    def _desenhar_mapa_referencias(self):
        if self.mapa is None:
            return
        self.mapa.delete_all_path()
        self.mapa.delete_all_marker()
        self.mapa.set_path(self.percurso.pontos, color=ACENTO, width=4)
        for r in self.lista_refs:
            if r and not r["saltado"]:
                p = self.percurso.ponto(r["d"])
                self.mapa.set_marker(p[0], p[1], icon=self._icone_referencia(), icon_anchor="center")

    def _atualizar_referencias(self):
        self.tabela_refs.delete(*self.tabela_refs.get_children())
        for t, r in zip(self.grelha, self.lista_refs):
            if r is None:
                continue
            coord = "saltado" if r["saltado"] else f"{r['lat']:.5f}, {r['lon']:.5f}"
            dist = "—" if r["saltado"] else f"{r['d']:.0f} m"
            self.tabela_refs.insert("", "end", values=(f"{self._t_rel(t):.1f} s", coord, dist))
        if self.tabela_refs.get_children():
            self.tabela_refs.see(self.tabela_refs.get_children()[-1])
        total = len(self.grelha)
        if self.j < total:
            self.texto_instante.configure(text=f"Instante {self._t_rel(self.grelha[self.j]):.1f} s  "
                                               f"(referência {self.j + 1} de {total})")
        else:
            self.texto_instante.configure(text="Referências completas." if total else
                                          f"Este trecho é mais curto do que {self.intervalo_s:g} s: não há referências a marcar.")
        self.bt_seguinte.configure(state="normal" if self.j >= total else "disabled")

    def _guardar_referencias(self):
        feitas = [r for r in self.lista_refs if r]
        if not feitas:
            self.msg_passo.configure(text="Ainda não há referências para guardar.")
            return
        conteudo = {"intervalo_s": self.intervalo_s,
                    "referencias": [{"t_ns": r["t_ns"], "t_s": round(self._t_rel(r["t_ns"]), 3),
                                     "lat": r["lat"], "lon": r["lon"], "saltado": r["saltado"]} for r in feitas]}
        caminho = self._guardar_json("referencias", self.ordem[self.i_trecho], conteudo)
        if caminho:
            self.msg_passo.configure(text=f"Referências guardadas em {caminho}", text_color=TEXTO_2)

    def _carregar_referencias(self):
        dados, aviso = self._carregar_json("referencias", self.ordem[self.i_trecho])
        if dados is None:
            if aviso:
                self.msg_passo.configure(text=aviso, text_color="#F59E0B")
            return
        if abs(float(dados.get("intervalo_s", self.intervalo_s)) - self.intervalo_s) > 1e-6:
            self.msg_passo.configure(text=f"As referências foram marcadas a cada {dados['intervalo_s']:g} s e agora "
                                          f"o intervalo é {self.intervalo_s:g} s. Recomece com o mesmo intervalo.",
                                     text_color="#F59E0B")
            return
        por_t = {int(r["t_ns"]): r for r in dados.get("referencias", [])}
        self.lista_refs = [None] * len(self.grelha)
        for k, t in enumerate(self.grelha):
            r = por_t.get(t)
            if r is None:
                break                                   # as referências são marcadas por ordem
            if r.get("saltado"):
                self.lista_refs[k] = {"t_ns": t, "lat": None, "lon": None, "d": None, "saltado": True}
            else:
                d = self.percurso.projetar((r["lat"], r["lon"]))[0]
                self.lista_refs[k] = {"t_ns": t, "lat": r["lat"], "lon": r["lon"], "d": d, "saltado": False}
        self.j = next((k for k, r in enumerate(self.lista_refs) if r is None), len(self.grelha))
        self.msg_passo.configure(text=aviso or f"Referências carregadas ({self.j} de {len(self.grelha)}).",
                                 text_color="#F59E0B" if aviso else TEXTO_2)
        self._desenhar_mapa_referencias()
        self._atualizar_referencias()
        self._esperar_imagem()

    def _fim_referencias(self):
        self.refs[self.ordem[self.i_trecho]] = [r for r in self.lista_refs if r]
        if self.i_trecho + 1 < len(self.ordem):
            self._passo_trajeto(self.i_trecho + 1)
        else:
            self._preparar_reproducao()

    # ---------- passo 3: mapa, vídeo e intervalos lado a lado ----------
    def _intervalos(self, n):
        """
        Intervalos entre referências de um trecho: [{"t0", "t1", "d0", "d1", "lat", "lon", "veic_media",
        "veic_max", "cor", "drift"}], com o início e o fim do trecho ligados ao início e ao fim do trajeto.
        """
        t = self.rep.trecho(n)
        perc = Percurso(self.trajetos[n])
        marcos = [(t["inicio_ns"], 0.0, self.trajetos[n][0])]
        for r in self.refs.get(n, []):
            if not r["saltado"]:
                marcos.append((r["t_ns"], r["d"], (r["lat"], r["lon"])))
        marcos.append((t["fim_ns"], perc.total, self.trajetos[n][-1]))
        dur = (t["fim_ns"] - t["inicio_ns"]) / 1e9
        v_media = perc.total / dur if dur > 0 else 0.0           # velocidade média do trecho (m/s)
        # veículos em cada instante (com duas câmaras, a soma das duas)
        instantes = registo.veiculos_por_instante(self.rep.deteccoes, t["inicio_ns"], t["fim_ns"] + 1)
        intervalos = []
        for k_int, ((t0, d0, p0), (t1, d1, _)) in enumerate(zip(marcos, marcos[1:])):
            ultimo = k_int == len(marcos) - 2                    # o último inclui o fim do trecho
            veic = [n for k, n in instantes if t0 <= k < t1 or (ultimo and k == t1)]
            media = sum(veic) / len(veic) if veic else 0.0
            intervalos.append({"t0": t0, "t1": t1, "d0": d0, "d1": d1, "lat": p0[0], "lon": p0[1],
                               "veic_media": media, "veic_max": max(veic) if veic else 0,
                               "cor": cor_da_gama(round(media)),
                               "drift": d0 + v_media * (t1 - t0) / 1e9 - d1})
        return intervalos, perc, v_media

    def _preparar_reproducao(self):
        self._limpar()
        topo = ctk.CTkFrame(self, fg_color="transparent")
        topo.pack(fill="both", expand=True)
        # com duas câmaras o vídeo (lado a lado) é mais largo: fica com mais espaço e o mapa com menos
        larguras = (26, 48, 26) if self.rep.duas else (37, 37, 26)
        topo.grid_columnconfigure(0, weight=larguras[0], uniform="col")    # mapa
        topo.grid_columnconfigure(1, weight=larguras[1], uniform="col")    # vídeo
        topo.grid_columnconfigure(2, weight=larguras[2], uniform="col")    # intervalos
        topo.grid_rowconfigure(0, weight=1)
        esq = ctk.CTkFrame(topo, fg_color="transparent")
        esq.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        widget, self.mapa = self._area_mapa(esq, tentar_de_novo=self._preparar_reproducao)
        widget.pack(fill="both", expand=True)

        meio = ctk.CTkFrame(topo, fg_color=CARTAO_2, corner_radius=10)
        meio.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        self.titulo_video = ctk.CTkLabel(meio, text="", font=ctk.CTkFont(size=15, weight="bold"), text_color=TEXTO, anchor="w")
        self.titulo_video.pack(fill="x", padx=14, pady=(10, 4))
        self.ecra = tk.Label(meio, bg="#000000", bd=0)
        self.ecra.pack(fill="both", expand=True, padx=14)
        self.progresso = ctk.CTkProgressBar(meio, progress_color=ACENTO, height=6)
        self.progresso.set(0)
        self.progresso.pack(fill="x", padx=14, pady=(8, 2))
        self.tempo = ctk.CTkLabel(meio, text="", text_color=TEXTO_2, anchor="w")
        self.tempo.pack(fill="x", padx=14, pady=(0, 10))

        lado = ctk.CTkFrame(topo, fg_color=CARTAO_2, corner_radius=10)
        lado.grid(row=0, column=2, sticky="nsew")
        ctk.CTkLabel(lado, text="Intervalos", font=ctk.CTkFont(size=15, weight="bold"), text_color=TEXTO,
                     anchor="w").pack(fill="x", padx=12, pady=(10, 4))
        self.tabela_int = ttk.Treeview(lado, columns=("n", "t", "ref", "v", "drift"), show="headings")
        for col, txt, larg in (("n", "#", 24), ("t", "Tempo (s)", 66), ("ref", "Referência (início)", 128),
                               ("v", "Veíc.", 40), ("drift", "Drift", 48)):
            self.tabela_int.heading(col, text=txt)
            self.tabela_int.column(col, width=self.app.px(larg), minwidth=self.app.px(larg), anchor="center",
                                   stretch=col == "ref")
        self.tabela_int.pack(fill="both", expand=True, padx=12)
        self.resumo_percurso = ctk.CTkLabel(lado, text="", text_color=TEXTO_2, anchor="w", justify="left",
                                            font=ctk.CTkFont(size=12))
        self.resumo_percurso.pack(fill="x", padx=12, pady=(8, 12))

        baixo = ctk.CTkFrame(self, fg_color="transparent")
        baixo.pack(fill="x", pady=(8, 0))
        self.bt_run = ctk.CTkButton(baixo, text="Run", width=120, fg_color=ACENTO, command=self._alternar)
        self.bt_run.pack(side="left")
        ctk.CTkButton(baixo, text="Recomeçar", width=110, fg_color="transparent", border_width=1,
                      border_color=TEXTO_3, text_color=TEXTO_2, hover_color=CARTAO,
                      command=self.reiniciar).pack(side="left", padx=8)
        legenda = ctk.CTkFrame(baixo, fg_color="transparent")
        legenda.pack(side="right")
        ctk.CTkLabel(legenda, text="Média de veículos (YOLO" + (", soma das câmaras" if self.rep.duas else "") + "):",
                     text_color=TEXTO_2).pack(side="left", padx=(0, 8))
        for _, _, rotulo, cor in GAMAS_VEICULOS:
            ctk.CTkFrame(legenda, width=22, height=5, corner_radius=2, fg_color=cor).pack(side="left")
            ctk.CTkLabel(legenda, text=rotulo, text_color=TEXTO).pack(side="left", padx=(5, 14))
        ctk.CTkFrame(legenda, width=22, height=5, corner_radius=2, fg_color=COR_PERCURSO_POR_FAZER).pack(side="left")
        ctk.CTkLabel(legenda, text="por percorrer / drift", text_color=TEXTO_2).pack(side="left", padx=(5, 0))
        self._iniciar_trecho(0, correr=False)

    def _preencher_intervalos(self):
        estilo = ttk.Style(self)
        self.tabela_int.delete(*self.tabela_int.get_children())
        for a, b, rotulo, cor in GAMAS_VEICULOS:
            self.tabela_int.tag_configure(rotulo, background=misturar(cor, CARTAO_2, 0.28))
        self.tabela_int.tag_configure("atual", font=("Segoe UI", 10, "bold"), foreground="#FFFFFF")
        self.tabela_int.tag_configure("feito", foreground=TEXTO)
        self.tabela_int.tag_configure("por_fazer", foreground=TEXTO_3)
        self.linhas_int = []
        for k, it in enumerate(self.intervalos, 1):
            rotulo = next(r for a, b, r, c in GAMAS_VEICULOS if c == it["cor"])
            ident = self.tabela_int.insert("", "end", tags=(rotulo, "por_fazer"), values=(
                k, f"{self._t_rel(it['t0']):.1f}–{self._t_rel(it['t1']):.1f}", f"{it['lat']:.5f}, {it['lon']:.5f}",
                f"{it['veic_media']:.1f}", f"{it['drift']:+.0f} m"))
            self.linhas_int.append(ident)
        dur = (self.trecho["fim_ns"] - self.trecho["inicio_ns"]) / 1e9
        todas = [n for _, n in registo.veiculos_por_instante(self.rep.deteccoes, self.trecho["inicio_ns"],
                                                             self.trecho["fim_ns"] + 1)]
        drifts = [abs(it["drift"]) for it in self.intervalos[:-1]] or [0.0]
        n_ref = len([r for r in self.refs.get(self.trecho["n"], []) if not r["saltado"]])
        n_salt = len([r for r in self.refs.get(self.trecho["n"], []) if r["saltado"]])
        self.resumo_percurso.configure(text=(
            f"Trecho {self.trecho['n']}:  {self.percurso.total:.0f} m em {tempo_txt(dur)}\n"
            f"Velocidade média: {self.v_media * 3.6:.0f} km/h\n"
            f"Veículos por {'instante (2 câmaras)' if self.rep.duas else 'imagem'}: "
            f"{sum(todas) / len(todas) if todas else 0:.1f} (máx. {max(todas) if todas else 0})\n"
            f"Referências: {n_ref}" + (f" ({n_salt} saltada{'s' if n_salt != 1 else ''})" if n_salt else "") + "\n"
            f"Drift entre referências: médio {sum(drifts) / len(drifts):.0f} m, máximo {max(drifts):.0f} m\n"
            "(+ a estimativa passou a referência seguinte;\n − ficou aquém: troço cinzento no mapa)"))

    def _iniciar_trecho(self, k, correr):
        """Prepara o trecho k da ordem: trajeto no mapa (cinzento), intervalos e leitura das imagens."""
        self.parar()
        self.k = k
        n = self.ordem[k]
        self.trecho = self.rep.trecho(n)
        self.intervalos, self.percurso, self.v_media = self._intervalos(n)
        if self.mapa is not None:
            self.mapa.delete_all_path()
            self.mapa.delete_all_marker()
            self._enquadrar(self.percurso.pontos)
            self.mapa.set_path(self.percurso.pontos, color=COR_PERCURSO_POR_FAZER, width=4)
        self._preencher_intervalos()
        self.i_int, self.seg_obj, self.seg_pts = -1, None, []
        self.titulo_video.configure(text=f"Trecho {n}" + (f"  ({k + 1} de {len(self.ordem)})" if len(self.ordem) > 1 else "")
                                    + (f"  ·  câmaras {' | '.join(self.rep.camaras)}" if self.rep.duas else
                                       (f"  ·  câmara {self.rep.camaras[0]}" if len(self.rep.camaras_registo) > 1 else "")))
        self.progresso.set(0)
        self.tempo.configure(text=f"00:00.0 / {tempo_txt((self.trecho['fim_ns'] - self.trecho['inicio_ns']) / 1e9)}")
        # Leitura num fio à parte (bag + desenho), para a janela não bloquear
        self.fila = queue.Queue(maxsize=8)
        self._parar_fio = threading.Event()
        self._fio = threading.Thread(target=self._ler, args=(self.trecho, self.fila, self._parar_fio), daemon=True)
        self._fio.start()
        self._pendente = None
        self.t_ant = None
        self.fim_trecho = False
        if correr:
            self._alternar()
        else:
            self.after(300, self._mostrar_primeira)     # depois de a janela ter o tamanho final

    def _ler(self, trecho, fila, parar):
        try:
            for item in self.rep.imagens(trecho):
                while not parar.is_set():
                    try:
                        fila.put(item, timeout=0.1)
                        break
                    except queue.Full:
                        pass
                if parar.is_set():
                    return
        except Exception as e:   # bag ilegível, etc.
            fila.put(("erro", str(e), None))
            return
        fila.put(None)

    def _mostrar_primeira(self):
        """Mostra a primeira imagem do trecho (antes de carregar em Run)."""
        try:
            item = self.fila.get(timeout=5)
        except queue.Empty:
            return
        self._pendente = item
        if item and item[0] != "erro":
            self._mostrar_imagem(item[1])

    def _alternar(self):
        if self.fim_trecho and self.k == len(self.ordem) - 1:
            self._iniciar_trecho(0, correr=True)      # «Repetir»
            return
        self.a_correr = not self.a_correr
        self.bt_run.configure(text="Parar" if self.a_correr else "Run")
        if self.a_correr:
            self.t_ant = None
            self._passo()

    def _passo(self):
        if not self.a_correr:
            return
        t0 = time.perf_counter()
        item, self._pendente = self._pendente, None
        if item is None:
            try:
                item = self.fila.get(timeout=1.0)
            except queue.Empty:
                self._apos = self.after(10, self._passo)
                return
        if item is None:                                  # fim do trecho
            self.fim_trecho = True
            if self.k + 1 < len(self.ordem):
                self._apos = self.after(800, lambda: self._iniciar_trecho(self.k + 1, correr=True))
            else:
                self.a_correr = False
                self.bt_run.configure(text="Repetir")
            return
        if item[0] == "erro":
            self.tempo.configure(text=f"Erro a ler o bag: {item[1]}")
            self.a_correr = False
            return
        t_ns, imagem, dados = item
        self._mostrar_imagem(imagem)
        self._avancar_mapa(t_ns)
        dur = (self.trecho["fim_ns"] - self.trecho["inicio_ns"]) / 1e9
        dec = (t_ns - self.trecho["inicio_ns"]) / 1e9
        self.progresso.set(dec / dur if dur > 0 else 1)
        self.tempo.configure(text=f"{tempo_txt(dec)} / {tempo_txt(dur)}")
        # ritmo da gravação
        intervalo = (t_ns - self.t_ant) / 1e9 if self.t_ant else 1 / 30
        intervalo = intervalo if 0 < intervalo < 1 else 1 / 30
        self.t_ant = t_ns
        espera = max(1, int((intervalo - (time.perf_counter() - t0)) * 1000))
        self._apos = self.after(espera, self._passo)

    def _mostrar_imagem(self, imagem_bgr):
        self._mostrar_em(self.ecra, imagem_bgr, max(320, self.ecra.winfo_width()), max(240, self.ecra.winfo_height()))

    def _avancar_mapa(self, t_ns):
        """
        Pinta o intervalo em curso com a sua cor. A posição parte da referência do início do intervalo e
        avança à velocidade média do trecho; nunca passa a referência seguinte. Ao mudar de intervalo,
        salta para a referência seguinte: o que ficou por pintar mostra o drift.
        """
        k = next((i for i, it in enumerate(self.intervalos) if it["t0"] <= t_ns < it["t1"]), len(self.intervalos) - 1)
        it = self.intervalos[k]
        if k != self.i_int:                              # novo intervalo: salta para a sua referência
            for j, ident in enumerate(self.linhas_int):  # destaca o intervalo em curso na tabela
                cor = self.tabela_int.item(ident, "tags")[0]
                estado = "atual" if j == k else ("feito" if j < k else "por_fazer")
                self.tabela_int.item(ident, tags=(cor, estado))
            self.i_int, self.seg_obj, self.seg_pts = k, None, []
            self.tabela_int.see(self.linhas_int[k])
        if self.mapa is None:
            return
        d = min(it["d0"] + self.v_media * (t_ns - it["t0"]) / 1e9, it["d1"], self.percurso.total)
        pontos = self.percurso.entre(it["d0"], d)
        if len(pontos) < 2 or d <= it["d0"]:
            return
        if self.seg_obj is None:
            self.seg_obj = self.mapa.set_path(pontos, color=it["cor"], width=6)
        else:
            self.seg_obj.set_position_list(pontos)


def main():
    parser = argparse.ArgumentParser(description="Resultados dos registos de deteções (resumo e percurso).")
    parser.add_argument("ensaio", nargs="?", default=None, help="Registo a mostrar primeiro no Resumo (opcional)")
    parser.add_argument("--pasta-registo", default=str(Path(__file__).resolve().parent / "data_offline"),
                        help="Pasta dos registos (por omissão: data_offline, na pasta do projeto)")
    args = parser.parse_args()
    App(args.pasta_registo, args.ensaio).mainloop()


if __name__ == "__main__":
    main()
