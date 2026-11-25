# -*- coding: utf-8 -*-
"""
anotar_referencia.py - Marcar à mão a referência (objetos, linhas, área circulável e faixa ao lado)
em imagens escolhidas ao acaso dos bags, para medir o desempenho da deteção (metricas_referencia.py).
README, secção 4.11.

  python offline/anotar_referencia.py <nome> --bag <bag.mcap> [--bag <outro.mcap>] [--n 60]   criar e marcar
  python offline/anotar_referencia.py <nome>                                                 continuar a marcar
  python offline/anotar_referencia.py <nome> --imagem 12                                     ir à imagem 12

As imagens aparecem corrigidas da distorção (as que os modelos recebem) e SEM as deteções do
processamento (para a marcação não ser influenciada por elas). O desenho usa as mesmas cores do
processamento (visual.py): objetos pela cor da classe, linhas a amarelo, área circulável a verde.

Modos: O objetos, L linhas, E estrada (área circulável). Em cada imagem marca-se TUDO o que se vê
dos três (o que não se marcar conta como "não existe"); uma imagem que não sirva (desfocada,
tapada) marca-se com I (ignorar).
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
import referencia  # noqa: E402
from va_g05 import geometria, visual  # noqa: E402

JANELA = "Referencia - G5"
LARGURA_PAINEL = 330
NOMES_PT = {"person": "peão", "bicycle": "bicicleta", "car": "carro", "motorcycle": "mota", "bus": "autocarro",
            "truck": "camião", "traffic light": "semáforo", "stop sign": "sinal STOP"}
MODOS = {"objetos": "O", "linhas": "L", "estrada": "E"}


def distancia_linha(pontos, x, y):
    """Distância (px) de (x, y) à polilinha."""
    p = np.asarray(pontos, dtype=np.float64)
    a, b = p[:-1], p[1:]
    ab = b - a
    t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) / np.maximum((ab ** 2).sum(1), 1e-9), 0, 1)
    q = a + ab * t[:, None]
    return float(np.min(np.hypot(q[:, 0] - x, q[:, 1] - y)))


class Anotador:
    def __init__(self, amostra, inicio=0):
        self.amostra = amostra
        self.nome = amostra["nome"]
        self.lista = amostra["imagens"]
        self.i = inicio
        self.modo = "objetos"
        self.classe = "car"
        self.buraco = False             # estrada: o polígono seguinte tira área (por baixo de um veículo, etc.)
        self.pontos = []                # linha ou polígono em curso
        self.arrasto = None             # caixa em curso: (x0, y0, x1, y1)
        self.ver = True                 # mostrar a marcação (V esconde, para ver a imagem limpa)
        self.alterada = False
        montagem = geometria.carregar_montagem() or {}
        self.lado = {c: v.get("lado") for c, v in (montagem.get("camaras") or {}).items()}
        self.escala = 2.0
        self.abrir()

    # ------------------------------------------------------------------ imagem
    def abrir(self):
        self.im = self.lista[self.i]
        self.img = referencia.imagem(self.amostra, self.im)
        if self.img is None:
            self.img = visual.imagem_espera(640, 480, "Imagem em falta", "o bag não foi encontrado")
        self.h, self.w = self.img.shape[:2]
        self.escala = max(1.0, min(2.5, 1100 / self.w, 820 / self.h))
        self.a = referencia.ler_anotacao(self.nome, self.im)
        self.pontos, self.arrasto, self.alterada = [], None, False

    def gravar(self):
        self.terminar_poligono()
        referencia.gravar_anotacao(self.nome, self.a, self.w, self.h, self.amostra.get("corrigida", False))
        self.alterada = False

    def sair(self):
        """Ao sair só se grava a imagem atual se foi alterada (vê-la não conta como marcada)."""
        self.terminar_poligono()
        if self.alterada:
            self.gravar()

    def marcada(self, im):
        return referencia.ficheiro_anotacao(self.nome, im["id"]).exists()

    # ------------------------------------------------------------------ edição
    def img_xy(self, x, y):
        return [round(min(max(x / self.escala, 0), self.w - 1), 1), round(min(max(y / self.escala, 0), self.h - 1), 1)]

    def terminar_poligono(self):
        if self.modo == "linhas" and len(self.pontos) >= 2:
            self.a["linhas"].append(self.pontos)
            self.alterada = True
        elif self.modo == "estrada" and len(self.pontos) >= 3:
            self.a["estrada_buracos" if self.buraco else "estrada_contornos"].append(self.pontos)
            self.alterada = True
        self.pontos = []

    def apagar_em(self, x, y):
        """Botão direito sem nada em curso: apaga o elemento do modo atual debaixo do rato."""
        if self.modo == "objetos":
            for o in reversed(self.a["objetos"]):
                x1, y1, x2, y2 = o["caixa"]
                if x1 <= x <= x2 and y1 <= y <= y2:
                    self.a["objetos"].remove(o)
                    self.alterada = True
                    return
        elif self.modo == "linhas":
            melhor, d_min = None, 12.0
            for ln in self.a["linhas"]:
                d = distancia_linha(ln, x, y)
                if d < d_min:
                    melhor, d_min = ln, d
            if melhor is not None:
                self.a["linhas"].remove(melhor)
                self.alterada = True
        else:
            for chave in ("estrada_buracos", "estrada_contornos"):
                for p in reversed(self.a[chave]):
                    if cv2.pointPolygonTest(np.array(p, np.float32).reshape(-1, 1, 2), (x, y), False) >= 0:
                        self.a[chave].remove(p)
                        self.alterada = True
                        return

    def desfazer(self):
        if self.pontos:
            self.pontos.pop()
            return
        chave = {"objetos": "objetos", "linhas": "linhas"}.get(self.modo)
        if chave is None:
            chave = "estrada_buracos" if self.buraco and self.a["estrada_buracos"] else "estrada_contornos"
        if self.a[chave]:
            self.a[chave].pop()
            self.alterada = True

    def rato(self, evento, x, y, flags, _):
        px, py = self.img_xy(x, y)
        if self.modo == "objetos":
            if evento == cv2.EVENT_LBUTTONDOWN:
                self.arrasto = [px, py, px, py]
            elif evento == cv2.EVENT_MOUSEMOVE and self.arrasto:
                self.arrasto[2:] = [px, py]
            elif evento == cv2.EVENT_LBUTTONUP and self.arrasto:
                x0, y0, x1, y1 = self.arrasto
                self.arrasto = None
                if abs(x1 - x0) >= 4 and abs(y1 - y0) >= 4:
                    self.a["objetos"].append({"classe": self.classe,
                                              "caixa": [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]})
                    self.alterada = True
            elif evento == cv2.EVENT_RBUTTONDOWN:
                self.apagar_em(px, py)
        else:
            if evento == cv2.EVENT_LBUTTONDOWN:
                self.pontos.append([px, py])
            elif evento == cv2.EVENT_RBUTTONDOWN:
                if self.pontos:
                    self.terminar_poligono()
                else:
                    self.apagar_em(px, py)
        self.rato_xy = (x, y)

    def tecla(self, k):
        """Devolve False para sair."""
        c = chr(k).lower() if 0 <= k < 256 else ""
        if k in (27,) or c == "q":
            self.sair()
            return False
        if c in ("o", "l", "e"):
            self.terminar_poligono()
            self.modo = {"o": "objetos", "l": "linhas", "e": "estrada"}[c]
        elif c.isdigit() and 1 <= int(c) <= len(referencia.CLASSES):
            self.classe = referencia.CLASSES[int(c) - 1]
            self.modo = "objetos"
        elif k in (13, 10):
            self.terminar_poligono()
        elif c == "b" and self.modo == "estrada":
            self.terminar_poligono()
            self.buraco = not self.buraco
        elif c == "z":
            self.desfazer()
        elif c == "f":
            ordem = [None, *referencia.ESTADOS_FAIXA]
            self.a["faixa_ao_lado"] = ordem[(ordem.index(self.a.get("faixa_ao_lado")) + 1) % len(ordem)]
            self.alterada = True
        elif c == "i":
            self.a["ignorar"] = not self.a.get("ignorar")
            self.alterada = True
        elif c == "v":
            self.ver = not self.ver
        elif c in ("n", "d", " ", "p", "a", "g"):
            self.gravar()
            if c in ("n", "d", " "):
                self.i = min(self.i + 1, len(self.lista) - 1)
            elif c in ("p", "a"):
                self.i = max(self.i - 1, 0)
            else:                                    # g: a seguinte por marcar
                resto = [j for j in range(len(self.lista)) if not self.marcada(self.lista[j])]
                self.i = next((j for j in resto if j > self.i), resto[0] if resto else self.i)
            self.abrir()
        return True

    # ------------------------------------------------------------------ desenho
    def desenhar(self):
        img = self.img.copy()
        if self.ver:
            m = referencia.mascara(self.a, self.w, self.h)
            visual.desenhar_estrada(img, m)
            for p in self.a["estrada_buracos"]:
                cv2.polylines(img, [np.round(np.array(p)).astype(np.int32)], True, (60, 60, 200), 1, cv2.LINE_AA)
            visual.desenhar_linhas(img, referencia.linhas(self.a))
            visual.desenhar_caixas(img, referencia.caixas(self.a))
        img = cv2.resize(img, (round(self.w * self.escala), round(self.h * self.escala)), interpolation=cv2.INTER_LINEAR)
        s = self.escala
        if self.ver:
            for o in self.a["objetos"]:
                x1, y1 = o["caixa"][:2]
                cv2.putText(img, o["classe"], (int(x1 * s) + 3, int(y1 * s) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            visual._bgr(visual.cor_classe(o["classe"])), 1, cv2.LINE_AA)
        if self.arrasto:
            x0, y0, x1, y1 = (v * s for v in self.arrasto)
            cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), visual._bgr(visual.cor_classe(self.classe)), 1)
        if self.pontos:
            cor = visual._bgr(visual.COR_MODELO["linhas"]) if self.modo == "linhas" else (
                (60, 60, 200) if self.buraco else visual._bgr(visual.COR_MODELO["estrada"]))
            p = (np.array(self.pontos) * s).astype(np.int32)
            cv2.polylines(img, [p.reshape(-1, 1, 2)], self.modo == "estrada", cor, 2, cv2.LINE_AA)
            for q in p:
                cv2.circle(img, tuple(int(v) for v in q), 3, cor, -1)
        if self.a.get("ignorar"):
            img = (img * 0.4).astype(np.uint8)
        return np.hstack([img, self.painel(img.shape[0])])

    def painel(self, altura):
        base = Image.new("RGB", (LARGURA_PAINEL, altura), (20, 23, 28))
        d = ImageDraw.Draw(base)
        f_t, f, f_b = visual.fonte(17, True), visual.fonte(13), visual.fonte(13, True)
        y = 12

        def linha(txt, fonte=f, cor=(209, 213, 219), dy=19, x=14):
            nonlocal y
            d.text((x, y), txt, font=fonte, fill=cor)
            y += dy

        feitas = sum(self.marcada(im) for im in self.lista)
        linha(f"Imagem {self.i + 1} / {len(self.lista)}", f_t, (255, 255, 255), 24)
        linha(f"câmara {self.im['camara']}  ·  {self.im['bag']}", cor=(156, 163, 175))
        linha(f"t = {self.im['t_s']:.1f} s  ·  marcadas: {feitas}", cor=(156, 163, 175))
        estado = "IGNORADA (I)" if self.a.get("ignorar") else ("guardada" if self.marcada(self.im) and not self.alterada
                                                              else "por guardar")
        linha(estado, f_b, (239, 68, 68) if self.a.get("ignorar") else (156, 163, 175), 26)

        for modo, tecla in MODOS.items():
            ativo = modo == self.modo
            n = {"objetos": len(self.a["objetos"]), "linhas": len(self.a["linhas"]),
                 "estrada": len(self.a["estrada_contornos"])}[modo]
            cor = visual.COR_MODELO[modo]
            d.rectangle([14, y + 3, 24, y + 13], fill=cor)
            linha(f"[{tecla}] {modo}  ({n})" + ("   <" if ativo else ""), f_b if ativo else f,
                  (255, 255, 255) if ativo else (156, 163, 175), 20, x=32)
        y += 6
        if self.modo == "objetos":
            linha("arrastar: caixa  ·  dir.: apagar", cor=(156, 163, 175))
            for j, c in enumerate(referencia.CLASSES, 1):
                cor = visual.cor_classe(c)
                d.rectangle([14, y + 3, 24, y + 13], fill=cor)
                linha(f"[{j}] {NOMES_PT.get(c, c)}" + ("   <" if c == self.classe else ""),
                      f_b if c == self.classe else f, (255, 255, 255) if c == self.classe else (156, 163, 175), 19, x=32)
        elif self.modo == "linhas":
            linha("clique: ponto  ·  dir./Enter: fim da linha", cor=(156, 163, 175))
            linha("dir. (sem linha em curso): apagar", cor=(156, 163, 175))
            linha("cada linha pintada: uma linha", cor=(156, 163, 175))
        else:
            linha("clique: vértice  ·  dir./Enter: fechar", cor=(156, 163, 175))
            linha("dir. (sem polígono): apagar", cor=(156, 163, 175))
            linha("[B] buraco: " + ("SIM (tira área)" if self.buraco else "não"), f_b,
                  (239, 68, 68) if self.buraco else (156, 163, 175))
        y += 8
        lado = self.lado.get(self.im["camara"], "?")
        fa = self.a.get("faixa_ao_lado") or "por marcar"
        linha(f"[F] faixa ao lado ({lado}): {fa}", f_b, (255, 255, 255), 19)
        linha("OCUPADA: veículo na faixa ao lado,", cor=(156, 163, 175), dy=17)
        linha("do ângulo morto até ~20 m atrás", cor=(156, 163, 175), dy=24)
        for t in ("[N]/[Espaço] guardar e seguinte", "[P] guardar e anterior", "[G] guardar e ir à por marcar",
                  "[Z] desfazer  [V] ver/esconder", "[I] ignorar imagem  [Q] sair"):
            linha(t, cor=(156, 163, 175), dy=18)
        return cv2.cvtColor(np.asarray(base), cv2.COLOR_RGB2BGR)


def main():
    ap = argparse.ArgumentParser(description="Marcar a referência para as métricas de desempenho (README 4.11).")
    ap.add_argument("nome", help="nome do conjunto (pasta docs/referencia/<nome>)")
    ap.add_argument("--bag", action="append", default=[], help="bag de onde escolher as imagens (pode repetir)")
    ap.add_argument("--n", type=int, default=60, help="número de imagens a escolher (60)")
    ap.add_argument("--camara", action="append", default=None, help="só esta câmara (5 ou 8; pode repetir)")
    ap.add_argument("--semente", type=int, default=1, help="semente da escolha ao acaso (1)")
    ap.add_argument("--intervalo-min", type=float, default=2.0,
                    help="segundos mínimos entre duas imagens da mesma câmara (2)")
    ap.add_argument("--imagem", type=int, default=None, help="começar nesta imagem (1, 2, ...)")
    args = ap.parse_args()

    amostra = referencia.ler_amostra(args.nome)
    if args.bag:
        if amostra is not None:
            sys.exit(f"Erro: o conjunto '{args.nome}' já existe ({referencia.pasta(args.nome)}); "
                     "para continuar a marcar, corra sem --bag.")
        amostra = referencia.criar_amostra(args.nome, args.bag, args.n, args.camara, args.semente, args.intervalo_min)
        print(f"Conjunto criado: {referencia.pasta(args.nome)}  ({len(amostra['imagens'])} imagens)")
        referencia.extrair_imagens(amostra)
    elif amostra is None:
        sys.exit(f"Erro: o conjunto '{args.nome}' não existe. Crie-o com: "
                 f"python offline/anotar_referencia.py {args.nome} --bag <bag.mcap>")
    if not amostra["imagens"]:
        sys.exit("Erro: a amostra não tem imagens.")
    if not amostra.get("corrigida"):
        print("Aviso: sem calibração das câmaras: as imagens não são corrigidas (o processamento também não).")

    an = Anotador(amostra)
    if args.imagem:
        an.i = min(max(args.imagem - 1, 0), len(amostra["imagens"]) - 1)
    else:
        an.i = next((j for j, im in enumerate(amostra["imagens"]) if not an.marcada(im)), 0)
    an.abrir()
    print(f"Conjunto {args.nome}: {len(amostra['imagens'])} imagens; anotações em "
          f"{referencia.pasta(args.nome) / 'anotacoes'}")
    cv2.namedWindow(JANELA, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(JANELA, an.rato)
    while True:
        cv2.imshow(JANELA, an.desenhar())
        k = cv2.waitKey(30)
        if k != -1 and not an.tecla(k & 0xFF):
            break
        if cv2.getWindowProperty(JANELA, cv2.WND_PROP_VISIBLE) < 1:
            an.sair()
            break
    cv2.destroyAllWindows()
    feitas = sum(an.marcada(im) for im in amostra["imagens"])
    print(f"Marcadas: {feitas} de {len(amostra['imagens'])}. "
          f"Métricas: python offline/metricas_referencia.py {args.nome}")


if __name__ == "__main__":
    main()
