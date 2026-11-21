# -*- coding: utf-8 -*-
"""
resumo_online.py - Janela com o resumo de uma sessão de processamento em tempo real.

Aberta pelo processar_inter.sh no fim do processamento (só com janela; com -i não abre). Mostra o
mesmo que a tab Resumo do gui_resultados.py (offline), sem o percurso: sessão e câmaras, modelos,
linhas e área circulável, deteções por categoria e veículos. Com duas câmaras, escolhe-se «Ambas»,
«Câmara 5» ou «Câmara 8». Lê o resumo_online.json gravado pelo processar_tempo_real.py.

  python resumo_online.py data_realtime_inter/realtime_<data_hora>
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gui_comum
import registo
from gui_comum import ACENTO, CARTAO, FUNDO, TEXTO, TEXTO_2, TEXTO_3, cartao, ctk, linhas_chave_valor, tempo_txt


def intervalos_txt(intervalos):
    return ", ".join(f"{tempo_txt(a)}–{tempo_txt(b)}" for a, b in intervalos)


class ResumoOnline(ctk.CTk):
    def __init__(self, caminho):
        super().__init__()
        ctk.set_appearance_mode("dark")
        caminho = Path(caminho)
        self.ficheiro = caminho / "resumo_online.json" if caminho.is_dir() else caminho
        self.resumo = json.loads(self.ficheiro.read_text(encoding="utf-8"))
        self.title("Resumo do processamento em tempo real")
        self.geometry("1320x760")
        self.minsize(1000, 600)
        self.configure(fg_color=FUNDO)
        gui_comum.estilo_tabelas(self)

        cab = ctk.CTkFrame(self, fg_color="transparent")
        cab.pack(fill="x", padx=18, pady=(14, 0))
        ctk.CTkLabel(cab, text="Resumo do processamento em tempo real", font=ctk.CTkFont(size=20, weight="bold"),
                     text_color=TEXTO).pack(side="left")
        ctk.CTkLabel(cab, text=f"   {self.ficheiro.parent}", font=ctk.CTkFont(size=12),
                     text_color=TEXTO_3).pack(side="left", pady=(4, 0))
        ctk.CTkButton(cab, text="Fechar", width=100, fg_color=ACENTO, command=self.destroy).pack(side="right")

        quadro = ctk.CTkFrame(self, fg_color=CARTAO, corner_radius=10)
        quadro.pack(fill="both", expand=True, padx=14, pady=(8, 14))
        self.cams = [c["camara"] for c in self.resumo["camaras"]]
        if len(self.cams) > 1:
            barra = ctk.CTkFrame(quadro, fg_color="transparent")
            barra.pack(fill="x", padx=8, pady=(10, 4))
            ctk.CTkLabel(barra, text="Câmara", text_color=TEXTO_2).pack(side="left", padx=(4, 8))
            self.filtro = ctk.CTkSegmentedButton(barra, values=["Ambas"] + [f"Câmara {c}" for c in self.cams],
                                                 command=lambda _: self._mostrar())
            self.filtro.set("Ambas")
            self.filtro.pack(side="left")
        else:
            self.filtro = None
        self.corpo = ctk.CTkScrollableFrame(quadro, fg_color="transparent")
        self.corpo.pack(fill="both", expand=True, padx=4, pady=4)
        self._mostrar()

    def _mostrar(self):
        for w in self.corpo.winfo_children():
            w.destroy()
        r = self.resumo
        escolha = self.filtro.get() if self.filtro else self.cams[0]
        chave = "Ambas" if escolha == "Ambas" else escolha.replace("Câmara ", "")
        resumo = r["resumos"][chave]
        cams = [str(c) for c in resumo["camaras"]]
        duas = len(cams) > 1
        info = {c["camara"]: c for c in r["camaras"]}
        self.corpo.grid_columnconfigure((0, 1, 2), weight=1, uniform="c")

        # --- Sessão (equivalente ao «Vídeo e registo» do offline) ---
        c = cartao(self.corpo, "Sessão")
        c.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        def por_cam(f):
            return "  ·  ".join(f"{c_}: {f(info[c_])}" for c_ in cams) if duas else f(info[cams[0]])

        pares = [
            ("Câmaras" if duas else "Câmara", registo.texto_camaras(cams) + (
                "  (5 à esquerda, 8 à direita)" if duas else
                (f"  (da sessão com as câmaras {registo.texto_camaras(self.cams)})" if len(self.cams) > 1 else ""))),
            ("Início", r.get("inicio", "—").replace("T", " ")),
            ("Duração", tempo_txt(r.get("duracao_s"))),
            ("Correção", por_cam(lambda x: "sim" if x.get("correcao") and "desligada" not in x["correcao"]
                                 else ("não (--sem-correcao)" if x.get("correcao") else "não (sem calibração)"))),
            ("Imagens processadas", por_cam(lambda x: str(x["imagens_processadas"]))),
            ("Deteções por segundo", por_cam(lambda x: "—" if x.get("deteccoes_por_segundo") is None
                                             else f"{x['deteccoes_por_segundo']:.1f}")),
        ]
        desligadas = [(c_, info[c_]["desligada_s"]) for c_ in cams if info[c_].get("desligada_s")]
        for c_, ints in desligadas:
            pares.append((f"Câmara {c_} desligada", intervalos_txt(ints)))
        pares.append(("Gravação", self.ficheiro.parent.name))
        linhas_chave_valor(c, pares)

        # --- Modelos ---
        c = cartao(self.corpo, "Modelos")
        c.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
        mod, tempos = r.get("modelos", {}), r.get("tempo_medio_ms", {})
        pares = []
        for k in ("objetos", "linhas", "estrada"):
            m = mod.get(k, {})
            lim = m.get("confianca_minima")
            txt = f"limiar {lim:g}" if isinstance(lim, (int, float)) else "classe mais provável"
            txt += f"  ·  {m.get('dispositivo', '?')}"
            if k in tempos:
                txt += f"  ·  {tempos[k]:.0f} ms/imagem"
            pares.append((m.get("nome", k), txt))
        classes = mod.get("objetos", {}).get("classes")
        if isinstance(classes, list):
            pares.append(("Classes", ", ".join(classes)))
        if tempos.get("total"):
            pares.append(("Por imagem", f"{tempos['total']:.0f} ms (correção, três modelos e desenho)"))
        linhas_chave_valor(c, pares)

        # --- Linhas e área circulável, deteções por categoria e veículos (iguais ao offline) ---
        gui_comum.desenhar_deteccoes(self.corpo, resumo, mod)
        if duas and desligadas:
            ctk.CTkLabel(self.corpo, text="Enquanto uma câmara esteve desligada, os veículos por instante "
                                          "contam só a outra.", text_color=TEXTO_3,
                         font=ctk.CTkFont(size=11), anchor="w").grid(row=2, column=0, columnspan=3, sticky="w",
                                                                     padx=10, pady=(0, 8))


def main():
    parser = argparse.ArgumentParser(description="Resumo de uma sessão de processamento em tempo real.")
    parser.add_argument("sessao", help="Pasta da sessão (data_realtime_inter/realtime_<data_hora>) "
                                       "ou o seu resumo_online.json")
    args = parser.parse_args()
    caminho = Path(args.sessao)
    ficheiro = caminho / "resumo_online.json" if caminho.is_dir() else caminho
    if not ficheiro.exists():
        print(f"Não há resumo em '{caminho}' (o processamento não chegou a gravar o resumo_online.json).")
        sys.exit(1)
    ResumoOnline(caminho).mainloop()


if __name__ == "__main__":
    main()
