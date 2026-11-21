# -*- coding: utf-8 -*-
"""
resumo_comparacao.py - Resumo de uma sessão de comparação de modelos de linhas (comparar_linhas.py).

Janela simples, com o aspeto do Resumo do offline (gui_comum.py): o vídeo e o registo; para cada
modelo (CLRNet e UFLDv2), as linhas por imagem, as imagens sem linhas, a distribuição do número de
linhas e o tempo por imagem; a concordância entre os dois (mesmo número de linhas, linhas
encontradas pelos dois e a distância entre elas); e os instantes guardados nas pausas.
Filtro por trecho, como no offline. Aberta no fim do comparar_linhas.py (com janela).

  python resumo_comparacao.py                                  # a sessão mais recente (data_comparacao)
  python resumo_comparacao.py data_comparacao/<origem>/<sessão>
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import comparacao
import gui_comum
import registo
from comparacao import MODELOS, NOMES
from gui_comum import (ACENTO, CARTAO, CARTAO_2, FUNDO, TEXTO, TEXTO_2, TEXTO_3, cartao, ctk,
                       linhas_chave_valor, pct, tempo_txt)

COR_MODELO = {"clrnet": "#FACC15", "ufld": "#38BDF8"}       # só para as barras desta janela


def abrir_pasta(pasta):
    """Abre a pasta no explorador de ficheiros do sistema."""
    try:
        if sys.platform.startswith("win"):
            os.startfile(str(pasta))                       # noqa (só existe no Windows)
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(pasta)])
        else:
            subprocess.Popen(["xdg-open", str(pasta)])
    except Exception as e:
        print(f"Não foi possível abrir a pasta ({e}): {pasta}")


class ResumoComparacao(ctk.CTk):
    def __init__(self, pasta):
        super().__init__()
        ctk.set_appearance_mode("dark")
        self.pasta = comparacao.pasta_da_sessao(pasta)
        self.ensaio = registo.ler_ensaio(self.pasta) or {}
        caminho = self.pasta / self.ensaio.get("ficheiro_deteccoes", "deteccoes.mcap")
        self.det = comparacao.ler_deteccoes_modelos(caminho) if caminho.exists() else {m: {} for m in MODELOS}
        self.instantes = comparacao.listar_instantes(self.pasta)
        self.title("Comparação de modelos de linhas")
        self.geometry("1320x780")
        self.minsize(1000, 600)
        self.configure(fg_color=FUNDO)
        gui_comum.estilo_tabelas(self)

        cab = ctk.CTkFrame(self, fg_color="transparent")
        cab.pack(fill="x", padx=18, pady=(14, 0))
        ctk.CTkLabel(cab, text="Comparação de modelos de linhas: " + " vs ".join(NOMES[m] for m in MODELOS),
                     font=ctk.CTkFont(size=20, weight="bold"), text_color=TEXTO).pack(side="left")
        ctk.CTkLabel(cab, text=f"   {self.pasta}", font=ctk.CTkFont(size=12), text_color=TEXTO_3).pack(side="left", pady=(4, 0))
        ctk.CTkButton(cab, text="Fechar", width=100, fg_color=ACENTO, command=self.destroy).pack(side="right")

        quadro = ctk.CTkFrame(self, fg_color=CARTAO, corner_radius=10)
        quadro.pack(fill="both", expand=True, padx=14, pady=(8, 14))
        barra = ctk.CTkFrame(quadro, fg_color="transparent")
        barra.pack(fill="x", padx=8, pady=(10, 4))
        ctk.CTkLabel(barra, text="Trecho", text_color=TEXTO_2).pack(side="left", padx=(4, 8))
        trechos = self.ensaio.get("trechos", [])
        self.filtro = ctk.CTkSegmentedButton(barra, values=["Todos"] + [str(t["n"]) for t in trechos],
                                             command=lambda _: self._mostrar())
        self.filtro.set("Todos")
        self.filtro.pack(side="left")
        self.corpo = ctk.CTkScrollableFrame(quadro, fg_color="transparent")
        self.corpo.pack(fill="both", expand=True, padx=4, pady=4)
        self._mostrar()

    def _selecionado(self):
        escolha = self.filtro.get()
        trechos = self.ensaio.get("trechos", [])
        if escolha != "Todos":
            trechos = [t for t in trechos if str(t["n"]) == escolha]
        nums = {t["n"] for t in trechos}
        return escolha, trechos, {m: {t: d for t, d in self.det[m].items() if d.get("trecho") in nums} for m in MODELOS}

    def _barras(self, pai, titulo, valores, rotulos, cor):
        ctk.CTkLabel(pai, text=titulo, text_color=TEXTO_2, anchor="w", font=ctk.CTkFont(size=12)).pack(
            fill="x", padx=14, pady=(2, 2))
        for rot, fr in zip(rotulos, valores):
            lin = ctk.CTkFrame(pai, fg_color="transparent")
            lin.pack(fill="x", padx=14, pady=1)
            ctk.CTkLabel(lin, text=rot, text_color=TEXTO, width=90, anchor="w", font=ctk.CTkFont(size=12)).pack(side="left")
            b = ctk.CTkProgressBar(lin, progress_color=cor, fg_color=CARTAO, height=10)
            b.set(fr)
            b.pack(side="left", fill="x", expand=True, padx=8)
            ctk.CTkLabel(lin, text=pct(fr), text_color=TEXTO_2, width=40, font=ctk.CTkFont(size=12)).pack(side="left")

    def _mostrar(self):
        for w in self.corpo.winfo_children():
            w.destroy()
        e = self.ensaio
        escolha, trechos, sel = self._selecionado()
        self.corpo.grid_columnconfigure((0, 1, 2), weight=1, uniform="c")
        mods = e.get("modelos", {})
        proc = e.get("processamento", {})

        # --- Vídeo e registo ---
        c = cartao(self.corpo, "Vídeo e registo")
        c.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        fonte = e.get("fonte", {})
        calib = e.get("calibracao")
        dur = sum((t["fim_ns"] - t["inicio_ns"]) / 1e9 for t in trechos)
        n_img = len(sel[MODELOS[0]])
        linhas_chave_valor(c, [
            ("Entrada", f"{fonte.get('nome', '—')} ({fonte.get('tipo', '?')})"),
            ("Câmara", str(e.get("camara") or "—")),
            ("Correção", f"sim ({calib['origem']})" if calib else "não"),
            ("Processado em", (e.get("criado_em") or "—").replace("T", " ")),
            ("Trechos", "  ".join(f"{t['n']}: {t.get('inicio_s', 0):.1f}–{t.get('fim_s', 0):.1f} s"
                                  for t in e.get("trechos", [])) or "—"),
            ("Selecionado", ("todos os trechos" if escolha == "Todos" else f"trecho {escolha}")
             + f"  ·  {n_img} imagens  ·  {tempo_txt(dur)}"),
            ("Mesma imagem", "os dois modelos recebem exatamente a mesma imagem"
             + (" (corrigida da distorção)" if calib else "")),
        ])

        # --- Um cartão por modelo ---
        for col, m in enumerate(MODELOS, start=1):
            info = mods.get(m, {})
            est = registo.estatisticas(list(sel[m].values()))
            c = cartao(self.corpo, f"{NOMES[m]}  ·  {'esquerda' if col == 1 else 'direita'}")
            c.grid(row=0, column=col, sticky="nsew", padx=6, pady=6)
            pares = [("Modelo", info.get("descricao", NOMES[m]).replace(", ", "\n", 1)),
                     ("Dispositivo", info.get("dispositivo", "—"))]
            if m in proc.get("tempo_medio_ms", {}):
                pares.append(("Tempo médio", f"{proc['tempo_medio_ms'][m]:.0f} ms por imagem"))
            if info.get("confianca_minima") is not None:
                pares.append(("Confiança mínima", f"{info['confianca_minima']:g}"))
            if info.get("corte_topo") is not None:
                pares.append(("Corte do topo", f"{info['corte_topo']:.0%}"))
            li = est["linhas"]
            pares += [("Linhas por imagem", "—" if li["media"] is None else f"{li['media']:.2f}"),
                      ("Imagens sem linhas", pct(li["pct_sem_linhas"]))]
            linhas_chave_valor(c, pares)
            self._barras(c, "Número de linhas por imagem (% das imagens)", comparacao.distribuicao_linhas(sel[m]),
                         ["0 linhas", "1 linha", "2 linhas", "3 linhas", "4 linhas"], COR_MODELO[m])
            ctk.CTkFrame(c, fg_color="transparent", height=8).pack()

        # --- Concordância ---
        a, b = MODELOS
        conc = comparacao.concordancia(sel[a], sel[b])
        c = cartao(self.corpo, f"Concordância entre o {NOMES[a]} e o {NOMES[b]}")
        c.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=6, pady=6)
        if not conc["imagens"]:
            linhas_chave_valor(c, [("—", "sem imagens registadas por trechos (tecla R, --trechos ou --registar-tudo)")])
        else:
            dist = conc["distancia_media_px"]
            linhas_chave_valor(c, [
                ("Imagens comparadas", str(conc["imagens"])),
                ("Mesmo número de linhas", pct(conc["pct_mesmo_numero"])),
                ("Diferença média", f"{conc['diferenca_media_numero']:.2f} linhas por imagem"),
                (f"Linhas do {NOMES[a]} também no {NOMES[b]}",
                 f"{pct(conc['pct_a_encontradas_por_b'])}  ({conc['emparelhadas']} de {conc['linhas_a']})"),
                (f"Linhas do {NOMES[b]} também no {NOMES[a]}",
                 f"{pct(conc['pct_b_encontradas_por_a'])}  ({conc['emparelhadas']} de {conc['linhas_b']})"),
                ("Distância entre as mesmas linhas", "—" if dist is None else f"{dist:.1f} px em média"),
                (f"Só o {NOMES[a]} encontrou linhas", pct(conc["pct_so_a"])),
                (f"Só o {NOMES[b]} encontrou linhas", pct(conc["pct_so_b"])),
            ])
        ctk.CTkLabel(c, text=f"Duas linhas são \"a mesma\" quando a distância média entre elas (em x, na parte em que "
                             f"as duas existem) é de {comparacao.LIMIAR_PX} px ou menos, na imagem que os modelos "
                             "receberam. Não é uma medida de qualidade (não há linhas de referência marcadas à mão): "
                             "mostra onde os modelos concordam e onde diferem; os instantes guardados ajudam a ver quem tem razão.",
                     text_color=TEXTO_3, font=ctk.CTkFont(size=11), anchor="w", justify="left",
                     wraplength=gui_comum.px(self, 760)).pack(fill="x", padx=14, pady=(0, 12))

        # --- Instantes guardados (pausas) ---
        c = cartao(self.corpo, f"Instantes guardados ({len(self.instantes)})")
        c.grid(row=1, column=2, sticky="nsew", padx=6, pady=6)
        if not self.instantes:
            ctk.CTkLabel(c, text="Nenhum. Cada pausa (Espaço) no comparar_linhas.py guarda o instante.",
                         text_color=TEXTO_2, wraplength=320, justify="left", anchor="w").pack(fill="x", padx=14, pady=(0, 12))
        for pasta, info in self.instantes:
            i = info.get("instante", {})
            n_lin = {m: len(info.get("deteccoes", {}).get(m, {}).get("linhas", [])) for m in MODELOS}
            lin = ctk.CTkFrame(c, fg_color=CARTAO, corner_radius=6)
            lin.pack(fill="x", padx=14, pady=2)
            txt = (f"{tempo_txt(i.get('segundos'))}  ·  " + "  ".join(f"{NOMES[m]} {n_lin[m]}" for m in MODELOS)
                   + (f"  ·  trecho {i['trecho']}" if i.get("trecho") else ""))
            ctk.CTkLabel(lin, text=txt, text_color=TEXTO, anchor="w", font=ctk.CTkFont(size=12)).pack(
                side="left", padx=8, pady=4)
            ctk.CTkButton(lin, text="Abrir", width=60, height=24, fg_color=CARTAO_2, hover_color="#2D3748",
                          command=lambda p=pasta: abrir_pasta(p)).pack(side="right", padx=6)
        if self.instantes:
            ctk.CTkButton(c, text="Abrir a pasta dos instantes", fg_color=CARTAO, hover_color="#2D3748",
                          command=lambda: abrir_pasta(self.pasta / "instantes")).pack(fill="x", padx=14, pady=(6, 12))


def abrir(pasta):
    ResumoComparacao(pasta).mainloop()


def main():
    ap = argparse.ArgumentParser(description="Resumo de uma sessão de comparação de modelos de linhas.")
    ap.add_argument("sessao", nargs="?", default=None,
                    help="Pasta da sessão (data_comparacao/<origem>/<sessão>). Sem nada: a mais recente")
    ap.add_argument("--pasta-registo", default="data_comparacao", help="Pasta das sessões (por omissão: data_comparacao)")
    args = ap.parse_args()
    if args.sessao:
        pasta = comparacao.pasta_da_sessao(args.sessao)
    else:
        sessoes = comparacao.listar_sessoes(args.pasta_registo)
        if not sessoes:
            print(f"Não há sessões em '{args.pasta_registo}'. Faça uma com: python comparar_linhas.py <bag ou vídeo>")
            sys.exit(1)
        pasta = sessoes[-1]
    if not (pasta / "ensaio.json").exists() and not (pasta / "instantes").exists():
        print(f"'{pasta}' não é uma sessão do comparar_linhas.py.")
        sys.exit(1)
    abrir(pasta)


if __name__ == "__main__":
    main()
