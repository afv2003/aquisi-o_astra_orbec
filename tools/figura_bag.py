# -*- coding: utf-8 -*-
"""
figura_bag.py - Figuras, GIF e métricas para o relatório e a apresentação, a partir de um bag do online
(sem ROS). Adaptado do tools/figura_bag.py do modelo da aula 10 (va-projeto-g00) para o G5: em vez da
grelha de um LiDAR, os avisos das câmaras laterais.

    pip install matplotlib            (no venv do offline; o resto já está no requirements.txt)
    python tools/figura_bag.py data/g05_camaras_<data_hora>
    python tools/figura_bag.py data/g05_camaras_<data_hora> --gif docs/img/demo.gif
    python tools/figura_bag.py data/g05_camaras_<data_hora> --instantes 4 --saida docs/img

Gera, em --saida (por omissão docs/img/), com o nome do bag:
  <bag>_avisos.png      estado de cada aviso ao longo do tempo (mudança de faixa e saída de faixa por
                        lado, colisão e planeador A*), com os trechos gravados
  <bag>_instantes.png   o ecrã do online (câmaras com as deteções, vista de cima e coluna dos avisos) em
                        alguns instantes, escolhidos pelos avisos mais graves
  --gif <ficheiro>      o mesmo ecrã animado, ao longo dos trechos
e escreve no terminal uma tabela (Markdown) de métricas para o README: imagens e deteções por segundo,
frequência dos tópicos e percentagem do tempo em cada estado de cada aviso.

O ecrã é refeito com o painel.py a partir do que ficou gravado (imagens em JPEG, deteções, grelha e
avisos), exatamente como o visualizador do online o mostrou (com o src/va_g05/config/, encontrado sozinho).
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "va_g05"))   # biblioteca comum (pacote va_g05)
from va_g05 import aviso_colisao      # noqa: E402
from va_g05 import caminhos           # noqa: E402
from va_g05 import aviso_faixa        # noqa: E402
from va_g05 import aviso_saida_faixa  # noqa: E402
from va_g05 import geometria          # noqa: E402
from va_g05 import grelha_local       # noqa: E402
from va_g05 import leitura_bag        # noqa: E402
from va_g05 import painel             # noqa: E402
from va_g05 import registo            # noqa: E402
from va_g05 import visual             # noqa: E402

ORDEM = {"colisao": ["LIVRE", "ATENCAO", "AVISO", "PERIGO"],
         "faixa": ["LIVRE", "SEM_FAIXA", "SEM_INFORMACAO", "OCUPADA"],
         "saida": ["SEM_LINHA", "OK", "A_APROXIMAR", "PERTO", "A_PISAR"]}
COR_ESTADO = {"LIVRE": "#22C55E", "OK": "#22C55E", "OCUPADA": "#EF4444", "PERIGO": "#EF4444",
              "A_PISAR": "#EF4444", "AVISO": "#F97316", "PERTO": "#F97316", "ATENCAO": "#FACC15",
              "A_APROXIMAR": "#FACC15", "SEM_FAIXA": "#FB923C", "SEM_INFORMACAO": "#9CA3AF",
              "SEM_LINHA": "#9CA3AF", "DESVIO": "#F97316", "BLOQUEADO": "#EF4444"}


def ler(bag, grupo):
    """Lê do bag os avisos, o planeador, os trechos e o número de mensagens de cada tópico."""
    t = {"faixa": f"/{grupo}/aviso_faixa", "saida": f"/{grupo}/aviso_saida_faixa",
         "colisao": f"/{grupo}/aviso_colisao", "planeador": f"/{grupo}/estado", "trechos": "/processamento/trechos"}
    indice = leitura_bag.bag_tem_indice(bag)
    inicio, _ = leitura_bag.intervalo_do_bag(bag)
    series = defaultdict(list)      # nome -> [(t_s, estado)]
    trechos, abertos = [], {}
    for m in leitura_bag.ler_mensagens(bag, list(t.values()), indice):
        inicio = inicio or m.log_time_ns
        ts = (m.log_time_ns - inicio) / 1e9
        if m.channel.topic == t["planeador"]:
            series["Planeador A*"].append((ts, (m.ros_msg.data.split() or ["?"])[0]))
            continue
        try:
            d = json.loads(m.ros_msg.data)
        except ValueError:
            continue
        if m.channel.topic == t["trechos"]:
            if d.get("evento") == "inicio":
                abertos[d["trecho"]] = (d["stamp_ns"] - inicio) / 1e9
            elif d.get("evento") == "fim" and d.get("trecho") in abertos:
                trechos.append((d["trecho"], abertos.pop(d["trecho"]), (d["stamp_ns"] - inicio) / 1e9))
        elif m.channel.topic == t["colisao"]:
            series["Colisão"].append((ts, d.get("estado", "LIVRE")))
        else:
            nome = "Mudança" if m.channel.topic == t["faixa"] else "Saída"
            for lado in aviso_faixa.LADOS:
                if lado in d:
                    series[f"{nome} ({lado[:3]}.)"].append((ts, d[lado]["estado"]))
    return series, trechos, inicio or 0


def figura_avisos(series, trechos, destino, titulo):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    nomes = [n for n in ("Mudança (esq.)", "Mudança (dir.)", "Saída (esq.)", "Saída (dir.)", "Colisão",
                         "Planeador A*") if series.get(n)]
    if not nomes:
        print("O bag não tem avisos gravados (gravado antes dos avisos, ou sem processamento).")
        return
    fim = max(v[-1][0] for v in series.values()) + 0.5
    fig, ax = plt.subplots(figsize=(12, 0.55 * len(nomes) + 1.6))
    for _, a, b in trechos:
        ax.axvspan(a, b, color="#E5E7EB", zorder=0)
    vistos = set()
    for i, nome in enumerate(nomes):
        pts = series[nome]
        for (t0, e), (t1, _) in zip(pts, pts[1:] + [(min(pts[-1][0] + 0.5, fim), None)]):
            if t1 - t0 > 2.0:               # intervalo entre trechos (nada gravado)
                t1 = t0 + 0.5
            ax.barh(i, t1 - t0, left=t0, height=0.6, color=COR_ESTADO.get(e, "#6B7280"), linewidth=0)
            vistos.add(e)
    ax.set_yticks(range(len(nomes)), nomes)
    ax.invert_yaxis()
    ax.set_xlabel("tempo desde o início da sessão (s); a cinzento, os trechos gravados")
    ax.set_title(titulo)
    ax.legend(handles=[Patch(color=COR_ESTADO.get(e, "#6B7280"), label=e.replace("_", " ")) for e in sorted(vistos)],
              loc="upper center", bbox_to_anchor=(0.5, -0.32 if len(nomes) > 3 else -0.5), ncol=6, fontsize=8,
              frameon=False)
    fig.tight_layout()
    fig.savefig(destino, dpi=130)
    plt.close(fig)
    print(f"Figura: {destino}")


class Ecra:
    """Refaz o ecrã do visualizador do online a partir das mensagens gravadas."""

    def __init__(self, bag, grupo, camaras):
        self.montagem = geometria.carregar_montagem()
        self.grelha = grelha_local.GrelhaLocal(self.montagem) if self.montagem else None
        self.p_faixa = aviso_faixa.parametros(self.montagem)
        self.camaras = camaras
        self.calib = {c: leitura_bag.carregar_calibracao_yaml(caminhos.calibracao(c))
                      for c in camaras if caminhos.calibracao(c).exists()}
        self.corretor = {c: leitura_bag.CorretorDistorcao(v) for c, v in self.calib.items()}
        self.imagens = {c: {} for c in camaras}
        self.ecra = {c: None for c in camaras}
        self.linhas_chao, self.geos = {}, {}
        self.g = self.faixa = self.saida = self.colisao = self.planeador = None
        self.objetos = {}
        self.t0 = None

    def mensagem(self, m, topico_cam, grupo):
        """Junta uma mensagem; devolve True quando uma câmara foi redesenhada."""
        top = m.channel.topic
        self.t0 = self.t0 or m.log_time_ns
        if top in topico_cam:
            c = topico_cam[top]
            self.imagens[c][m.log_time_ns] = m.ros_msg
            for t in [t for t in self.imagens[c] if t < m.log_time_ns - 3e9]:
                del self.imagens[c][t]
            return False
        if top == f"/{grupo}/grelha_local":
            g = m.ros_msg
            self.g = np.asarray(g.data, dtype=np.int8).reshape(int(g.info.height), int(g.info.width))
            return False
        if top == f"/{grupo}/estado":
            self.planeador = m.ros_msg.data
            return False
        try:
            d = json.loads(m.ros_msg.data)
        except ValueError:
            return False
        if top == f"/{grupo}/aviso_faixa":
            self.faixa = {k: v for k, v in d.items() if k in aviso_faixa.LADOS}
        elif top == f"/{grupo}/aviso_saida_faixa":
            self.saida = d
        elif top == f"/{grupo}/aviso_colisao":
            self.colisao = d
            for o in d.get("objetos", []):
                self.objetos[(str(o.get("camara")), o.get("stamp_ns"))] = [
                    x for x in d["objetos"] if str(x.get("camara")) == str(o.get("camara"))]
        elif top.endswith("/processamento/dados_deteccoes"):
            c = registo.camara_do_topico(top)
            img = self.imagens.get(c, {}).get(d.get("stamp_ns"))
            if img is None:
                return False
            frame = leitura_bag.imagem_bgr(img)
            corrigida = bool(d.get("corrigida")) and c in self.corretor
            if corrigida:
                frame = self.corretor[c](frame)
            h, w = frame.shape[:2]
            caixas, linhas, mascara = painel.deteccoes_de_json(d, w, h)
            valores = painel.valores(caixas, linhas, d.get("estrada_percentagem", 0.0))
            info = {"tempo": (d["stamp_ns"] - self.t0) / 1e9, "total": 0,
                    "subtitulo": f"Câmara {c}  ·  online (gravado)  ·  " + ("corrigida" if corrigida else "original"),
                    "cor_subtitulo": "#9CA3AF",
                    "modelos": [(k, painel.NOME_MODELO[k], valores[k], None, True, True) for k in painel.MODELOS],
                    "rodape": "refeito do bag (tools/figura_bag.py)"}
            self.ecra[c] = painel.camara(frame, caixas, linhas, mascara, info)
            painel.desenhar_alertas(self.ecra[c], self.objetos.get((c, d["stamp_ns"]), []), c, d["stamp_ns"])
            if self.grelha is not None and c in self.calib and c in self.montagem["camaras"]:
                k = geometria.matriz_intrinseca(self.calib[c], w, h, corrigida)
                self.linhas_chao[c] = self.grelha.linhas_no_chao(c, k, linhas)
                self.geos[c] = self.grelha.geometria(c, k)[0]
            return True
        return False

    def imagem(self):
        juntas = painel.juntar(self.camaras, self.ecra)
        if juntas is None:
            return None
        partes = [juntas]
        if self.grelha is not None and self.g is not None and self.g.shape == (self.grelha.altura, self.grelha.largura):
            partes.append(painel.vista_de_cima(self.grelha, self.g, juntas.shape[0], self.linhas_chao, self.geos,
                                               self.faixa, self.p_faixa,
                                               f"A*: {self.planeador}" if self.planeador else None, self.colisao))
            partes.append(painel.quadro_avisos(juntas.shape[0], self.faixa, self.saida, self.colisao,
                                               self.planeador or ""))
        return visual.lado_a_lado(partes)


def gravidade(e):
    """Pontuação de um ecrã (para escolher os instantes mais interessantes)."""
    p = aviso_colisao.NIVEL.get((e.colisao or {}).get("estado", "LIVRE"), 0) * 10
    p += sum(4 for v in (e.faixa or {}).values() if v.get("estado") == "OCUPADA")
    p += sum(aviso_saida_faixa.ORDEM.get(v.get("estado"), 0) for k, v in (e.saida or {}).items()
             if k in aviso_saida_faixa.LADOS)
    return p


def ecras(bag, grupo, instantes, gif, largura_gif, fps_gif):
    topicos = leitura_bag.topicos_do_bag(bag)
    cams = leitura_bag.camaras_no_bag(topicos)
    camaras = visual.ordenar_camaras(cams)
    topico_cam = {t: c for c, t in cams.items()}
    e = Ecra(bag, grupo, camaras)
    lidos = list(topico_cam) + [registo.topico_deteccoes(c) for c in camaras] + [
        f"/{grupo}/{n}" for n in ("grelha_local", "aviso_faixa", "aviso_saida_faixa", "aviso_colisao", "estado")]
    candidatos, quadros, t_gif = [], [], None
    for m in leitura_bag.ler_mensagens(bag, lidos, leitura_bag.bag_tem_indice(bag)):
        if not e.mensagem(m, topico_cam, grupo):
            continue
        img = e.imagem()
        if img is None:
            continue
        t_s = (m.log_time_ns - e.t0) / 1e9
        g = gravidade(e)
        if len(candidatos) < 40 or g > candidatos[-1][0]:
            candidatos = sorted(candidatos + [(g, t_s, img)], key=lambda c: -c[0])[:40]   # só os melhores
        if gif and (t_gif is None or t_s - t_gif >= 1.0 / fps_gif or t_s < t_gif):
            t_gif = t_s
            esc = largura_gif / img.shape[1]
            q = cv2.resize(img, (largura_gif, int(img.shape[0] * esc)), interpolation=cv2.INTER_AREA)
            quadros.append(cv2.imencode(".jpg", q, [cv2.IMWRITE_JPEG_QUALITY, 85])[1])   # comprimido (memória)
    escolhidos = []
    for g, t_s, img in candidatos:                 # os mais graves, com pelo menos 2 s entre eles
        if all(abs(t_s - t) >= 2.0 for _, t, _ in escolhidos):
            escolhidos.append((g, t_s, img))
        if len(escolhidos) == instantes:
            break
    return sorted(escolhidos, key=lambda c: c[1]), quadros


def guardar_instantes(escolhidos, destino):
    if not escolhidos:
        print("Sem ecrãs para mostrar (o bag não tem deteções gravadas?).")
        return
    linhas = []
    for _, t_s, img in escolhidos:
        img = img.copy()
        cv2.putText(img, f"t = {t_s:.1f} s", (10, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                    cv2.LINE_AA)
        linhas.append(img)
    largura = max(i.shape[1] for i in linhas)
    linhas = [cv2.copyMakeBorder(i, 0, 6, 0, largura - i.shape[1], cv2.BORDER_CONSTANT, value=(12, 12, 12))
              for i in linhas]
    cv2.imwrite(str(destino), cv2.vconcat(linhas))
    print(f"Figura: {destino}")


def guardar_gif(quadros, destino, fps):
    from PIL import Image
    if not quadros:
        print("Sem ecrãs para o GIF.")
        return
    passo = max(1, len(quadros) // 300)                # no máximo ~300 imagens
    imgs = [Image.fromarray(cv2.cvtColor(cv2.imdecode(q, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)) for q in quadros[::passo]]
    imgs[0].save(destino, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0, optimize=True)
    print(f"GIF: {destino} ({len(imgs)} imagens)")


def metricas(bag, series, trechos):
    """Tabela Markdown para o README: frequência dos tópicos (nos trechos) e % do tempo em cada estado."""
    from mcap.reader import make_reader
    with open(bag, "rb") as f:
        s = make_reader(f).get_summary()
    dur = sum(b - a for _, a, b in trechos) or None
    print("\n| Tópico | Mensagens | Por segundo (nos trechos) |\n|---|---|---|")
    if s:
        for i, c in sorted(s.channels.items(), key=lambda kv: kv[1].topic):
            n = s.statistics.channel_message_counts.get(i, 0)
            print(f"| `{c.topic}` | {n} | {n / dur:.1f} |" if dur else f"| `{c.topic}` | {n} | — |")
    print("\n| Aviso | " + " | ".join(["% do tempo em cada estado"]) + " |\n|---|---|")
    for nome, pts in series.items():
        cont = defaultdict(int)
        for _, e in pts:
            cont[e] += 1
        tot = sum(cont.values())
        print(f"| {nome} | " + ", ".join(f"{e.replace('_', ' ')} {n / tot:.0%}"
                                          for e, n in sorted(cont.items(), key=lambda kv: -kv[1])) + " |")
    print("\n(percentagens das imagens processadas em cada estado)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag", help="pasta da sessão do online (data/g05_camaras_<data_hora>) ou o .mcap")
    ap.add_argument("--grupo", default="g05")
    ap.add_argument("--saida", default="docs/img", help="pasta das figuras (docs/img)")
    ap.add_argument("--instantes", type=int, default=3, help="número de instantes na figura dos ecrãs (3)")
    ap.add_argument("--gif", default=None, help="ficheiro do GIF animado (ex.: docs/img/demo.gif)")
    ap.add_argument("--largura-gif", type=int, default=960, help="largura do GIF em píxeis (960)")
    ap.add_argument("--fps-gif", type=float, default=4.0, help="imagens por segundo no GIF (4)")
    a = ap.parse_args()
    bag = Path(a.bag)
    if bag.is_dir():
        bag = next(iter(sorted(bag.glob("*.mcap"))), None)
        if bag is None:
            raise SystemExit(f"Erro: '{a.bag}' não tem nenhum .mcap.")
    erro = leitura_bag.verificar_mcap(bag)
    if erro:
        raise SystemExit(f"Erro: {erro}")
    saida = Path(a.saida)
    saida.mkdir(parents=True, exist_ok=True)
    nome = bag.stem
    series, trechos, _ = ler(bag, a.grupo)
    figura_avisos(series, trechos, saida / f"{nome}_avisos.png", f"Avisos ao longo do tempo: {nome}")
    escolhidos, quadros = ecras(bag, a.grupo, a.instantes, a.gif, a.largura_gif, a.fps_gif)
    guardar_instantes(escolhidos, saida / f"{nome}_instantes.png")
    if a.gif:
        Path(a.gif).parent.mkdir(parents=True, exist_ok=True)
        guardar_gif(quadros, a.gif, a.fps_gif)
    metricas(bag, series, trechos)


if __name__ == "__main__":
    main()
