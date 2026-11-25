# -*- coding: utf-8 -*-
"""
metricas_referencia.py - Desempenho da deteção face à referência marcada à mão (anotar_referencia.py).
README, secção 4.11.

  python offline/metricas_referencia.py <nome>                      métricas + imagens de comparação
  python offline/metricas_referencia.py <nome> --figuras 3          e copia 3 comparações para docs/img
  python offline/metricas_referencia.py <nome> --sem-imagens        só as métricas

Em cada imagem marcada (e não ignorada) corre o MESMO processamento do offline (processar_bag.py):
correção da distorção com a calibração da amostra, objetos (YOLO, classes e confiança por omissão),
linhas (CLRNet) e área circulável (YOLOP); e, para a faixa ao lado, a mesma decisão local (decisao.py)
com essa imagem. Depois compara com a referência:

  Objetos     uma deteção certa (VP) é uma caixa com IoU >= 0,5 com uma caixa marcada da MESMA classe
              (cada caixa marcada só serve uma vez, pela ordem do IoU); as outras deteções são falsos
              positivos (FP) e as caixas marcadas que sobram são falsos negativos (FN).
              Precisão = VP/(VP+FP), recall = VP/(VP+FN), F1 = 2PR/(P+R); por classe e no total.
              Classificação: das caixas encontradas com IoU >= 0,5 (qualquer classe), a % com a classe certa.
  Linhas      como no CULane: cada linha desenhada com 30 px de largura numa imagem de 1640 px (aqui
              proporcional: ~12 px a 640 px) e a mesma linha se o IoU das duas faixas for >= 0,5;
              precisão, recall e F1.
  Estrada     área circulável: IoU, precisão e recall por píxel (soma de todas as imagens) e IoU médio.
  Faixa ao    o estado do aviso de mudança de faixa do lado da câmara (LIVRE / OCUPADA / SEM_FAIXA),
  lado        com só essa imagem (sem o seguimento entre imagens), contra o marcado: matriz de confusão
              e acerto.

Resultados: docs/referencia/<nome>/metricas.md (tabelas para o README e o relatório) e metricas.json;
imagens em data/referencia/<nome>/: <id>_original.png, <id>_referencia.png, <id>_processamento.png
e <id>_comparacao.png (as três lado a lado, com as mesmas cores).
"""
import argparse
import json
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import referencia  # noqa: E402
from va_g05 import caminhos, decisao, estrada, geometria, leitura_bag, linhas, objetos, visual  # noqa: E402
from va_g05.seguimento import CATEGORIA  # noqa: E402

IOU_OBJETOS = 0.5
IOU_LINHAS = 0.5
LARGURA_LINHA_CULANE = 30 / 1640     # largura da linha (fração da largura da imagem), como no CULane


# --------------------------------------------------------------------------- correspondências
def iou_caixas(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    uniao = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / uniao if uniao > 0 else 0.0


def emparelhar(ref, det, iou, limiar, mesma=None):
    """Pares (i_ref, i_det) com iou >= limiar, do maior para o menor, cada um usado uma só vez."""
    pares = sorted(((iou(r, d), i, j) for i, r in enumerate(ref) for j, d in enumerate(det)
                    if mesma is None or mesma(r, d)), reverse=True)
    usados_r, usados_d, res = set(), set(), []
    for v, i, j in pares:
        if v < limiar:
            break
        if i not in usados_r and j not in usados_d:
            usados_r.add(i)
            usados_d.add(j)
            res.append((i, j))
    return res


def mascaras_linhas(lns, w, h):
    larg = max(3, round(LARGURA_LINHA_CULANE * w))   # ~12 px a 640 px
    res = []
    for p in lns:
        m = np.zeros((h, w), np.uint8)
        cv2.polylines(m, [np.round(np.asarray(p)).astype(np.int32).reshape(-1, 1, 2)], False, 1, larg)
        res.append(m.astype(bool))
    return res


def iou_mascaras(a, b):
    uniao = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / uniao) if uniao else 0.0


def prf(vp, fp, fn):
    p = vp / (vp + fp) if vp + fp else None
    r = vp / (vp + fn) if vp + fn else None
    f = 2 * p * r / (p + r) if p and r else (0.0 if p is not None and r is not None else None)
    return p, r, f


# --------------------------------------------------------------------------- processamento
class Processamento:
    """Os três detetores e a decisão local, como no processar_bag.py (opções por omissão)."""

    def __init__(self):
        self.objetos = objetos.DetetorObjetos(confianca=0.25, classes=referencia.CLASSES)
        self.linhas = linhas.DetetorLinhas()
        self.estrada = estrada.DetetorEstrada()
        self.montagem = geometria.carregar_montagem()
        print(f"Modelos: objetos {self.objetos.dispositivo}, linhas {self.linhas.dispositivo}, "
              f"estrada {self.estrada.dispositivo}")

    def correr(self, img, cam, cal, corrigida, stamp_ns):
        caixas = self.objetos.detetar(img)
        lns = self.linhas.detetar(img)
        m = self.estrada.detetar(img)
        faixa = None
        if self.montagem and cal and cam in self.montagem["camaras"]:
            h, w = img.shape[:2]
            k = geometria.matriz_intrinseca(leitura_bag.calibracao_de_dict(cal), w, h, corrigida)
            r = decisao.DecisaoLocal(self.montagem).atualizar(cam, k, w, h, stamp_ns, m, caixas, lns)
            lado = self.montagem["camaras"][cam].get("lado")
            faixa = (r["faixa"].get(lado) or {}).get("estado")
        return caixas, lns, m, faixa


def desenho(img, caixas, lns, m, titulo):
    out = img.copy()
    visual.desenhar_estrada(out, m)
    visual.desenhar_linhas(out, lns)
    visual.desenhar_caixas(out, caixas)
    for x1, y1, _, _, nome, _ in caixas:
        cv2.putText(out, nome, (int(x1) + 2, int(y1) - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    visual._bgr(visual.cor_classe(nome)), 1, cv2.LINE_AA)
    return visual.etiqueta_canto(out, titulo)


# --------------------------------------------------------------------------- tabelas
def pct(v):
    return "—" if v is None else f"{100 * v:.1f} %"


def tabela_md(cabecalho, linhas_):
    s = "| " + " | ".join(cabecalho) + " |\n|" + "---|" * len(cabecalho) + "\n"
    return s + "".join("| " + " | ".join(str(c) for c in ln) + " |\n" for ln in linhas_)


def main():
    ap = argparse.ArgumentParser(description="Métricas de desempenho face à referência marcada (README 4.11).")
    ap.add_argument("nome", help="nome do conjunto (docs/referencia/<nome>)")
    ap.add_argument("--sem-imagens", action="store_true", help="não gravar as imagens de comparação")
    ap.add_argument("--figuras", type=int, default=0,
                    help="copiar N imagens de comparação para docs/img (as de maior desacordo nos objetos)")
    args = ap.parse_args()

    amostra = referencia.ler_amostra(args.nome)
    if amostra is None:
        sys.exit(f"Erro: o conjunto '{args.nome}' não existe ({referencia.pasta(args.nome)}).")
    marcadas = [im for im in amostra["imagens"] if referencia.ficheiro_anotacao(args.nome, im["id"]).exists()]
    anot = {im["id"]: referencia.ler_anotacao(args.nome, im) for im in marcadas}
    usadas = [im for im in marcadas if not anot[im["id"]].get("ignorar")]
    if not usadas:
        sys.exit("Erro: nenhuma imagem marcada (python offline/anotar_referencia.py " + args.nome + ").")
    print(f"Conjunto {args.nome}: {len(amostra['imagens'])} imagens, {len(marcadas)} marcadas, "
          f"{len(marcadas) - len(usadas)} ignoradas, {len(usadas)} usadas.")

    proc = Processamento()
    destino_img = referencia.pasta_imagens(args.nome)
    obj = defaultdict(Counter)                     # (câmara|"total") -> {classe: vp/fp/fn}
    cat = defaultdict(Counter)
    classif = Counter()
    confusao_classes = Counter()
    lin = defaultdict(Counter)
    est = defaultdict(Counter)
    ious_est = defaultdict(list)
    faixa = defaultdict(Counter)                   # câmara -> (ref, proc)
    desacordo = []
    for n, im in enumerate(usadas, 1):
        a = anot[im["id"]]
        cam = im["camara"]
        cal = (amostra.get("calibracao") or {}).get(cam)
        img = referencia.imagem(amostra, im)
        if img is None:
            print(f"Aviso: {im['id']}: imagem em falta; ignorada.")
            continue
        h, w = img.shape[:2]
        caixas_d, lns_d, m_d, faixa_d = proc.correr(img, cam, cal, amostra.get("corrigida", False), im["stamp_ns"])
        caixas_r, lns_r, m_r = referencia.caixas(a), referencia.linhas(a), referencia.mascara(a, w, h)
        print(f"\r  {n}/{len(usadas)}  {im['id']}", end="", flush=True)

        # objetos, por classe (mesma classe) e por categoria (veículo / peão / bicicleta / outros)
        pares = emparelhar(caixas_r, caixas_d, iou_caixas, IOU_OBJETOS, lambda r, d: r[4] == d[4])
        ok_r, ok_d = {i for i, _ in pares}, {j for _, j in pares}
        for chave in (cam, "total"):
            for i, r in enumerate(caixas_r):
                obj[chave][(r[4], "vp" if i in ok_r else "fn")] += 1
            for j, d in enumerate(caixas_d):
                if j not in ok_d:
                    obj[chave][(d[4], "fp")] += 1
        catg = lambda c: CATEGORIA.get(c[4], "sinalização")  # noqa: E731
        pares_c = emparelhar(caixas_r, caixas_d, iou_caixas, IOU_OBJETOS, lambda r, d: catg(r) == catg(d))
        okc_r, okc_d = {i for i, _ in pares_c}, {j for _, j in pares_c}
        for i, r in enumerate(caixas_r):
            cat["total"][(catg(r), "vp" if i in okc_r else "fn")] += 1
        for j, d in enumerate(caixas_d):
            if j not in okc_d:
                cat["total"][(catg(d), "fp")] += 1
        for i, j in emparelhar(caixas_r, caixas_d, iou_caixas, IOU_OBJETOS):
            certo = caixas_r[i][4] == caixas_d[j][4]
            classif["certas" if certo else "erradas"] += 1
            if not certo:
                confusao_classes[(caixas_r[i][4], caixas_d[j][4])] += 1
        desacordo.append((len(caixas_r) - len(ok_r) + len(caixas_d) - len(ok_d), im["id"]))

        # linhas (CULane)
        mr, md = mascaras_linhas(lns_r, w, h), mascaras_linhas(lns_d, w, h)
        pl = emparelhar(mr, md, iou_mascaras, IOU_LINHAS)
        for chave in (cam, "total"):
            lin[chave]["vp"] += len(pl)
            lin[chave]["fn"] += len(mr) - len(pl)
            lin[chave]["fp"] += len(md) - len(pl)

        # estrada (por píxel)
        m_d = m_d if m_d is not None else np.zeros((h, w), bool)
        for chave in (cam, "total"):
            est[chave]["vp"] += int(np.logical_and(m_r, m_d).sum())
            est[chave]["fp"] += int(np.logical_and(~m_r, m_d).sum())
            est[chave]["fn"] += int(np.logical_and(m_r, ~m_d).sum())
            if m_r.any() or m_d.any():
                ious_est[chave].append(iou_mascaras(m_r, m_d))

        # faixa ao lado
        if a.get("faixa_ao_lado"):
            faixa[cam][(a["faixa_ao_lado"], faixa_d or "SEM_INFORMACAO")] += 1

        if not args.sem_imagens:
            destino_img.mkdir(parents=True, exist_ok=True)
            orig = referencia.imagem(amostra, im)
            ref = desenho(orig, caixas_r, lns_r, m_r, "referência (à mão)")
            det = desenho(orig, caixas_d, lns_d, m_d, "processamento")
            o = visual.etiqueta_canto(orig.copy(), "original" + (" (corrigida)" if amostra.get("corrigida") else ""))
            cv2.imwrite(str(destino_img / f"{im['id']}_referencia.png"), ref)
            cv2.imwrite(str(destino_img / f"{im['id']}_processamento.png"), det)
            cv2.imwrite(str(destino_img / f"{im['id']}_comparacao.png"), visual.lado_a_lado([o, ref, det]))
    print()

    # --------------------------------------------------------------- resultados
    res = {"conjunto": args.nome, "imagens_usadas": len(usadas), "imagens_ignoradas": len(marcadas) - len(usadas),
           "criterios": {"iou_objetos": IOU_OBJETOS, "iou_linhas": IOU_LINHAS,
                         "largura_linha": "30 px a 1640 px (CULane), proporcional"},
           "objetos": {}, "categorias": {}, "linhas": {}, "estrada": {}, "faixa_ao_lado": {}}
    cams = sorted({im["camara"] for im in usadas})
    md = [f"# Métricas de desempenho: {args.nome}\n",
          f"{len(usadas)} imagens marcadas à mão (câmaras {', '.join(cams)}; {len(marcadas) - len(usadas)} ignoradas), "
          f"bags: {', '.join(b['nome'] for b in amostra['bags'])}. Gerado com `python offline/metricas_referencia.py "
          f"{args.nome}` (README 4.11).\n"]

    # objetos
    linhas_md = []
    classes = sorted({c for (c, _) in obj["total"]}, key=lambda c: referencia.CLASSES.index(c)
                     if c in referencia.CLASSES else 99)
    for c in classes + ["total"]:
        s = obj["total"]
        vp, fp, fn = ((sum(v for (k, t), v in s.items() if t == x) for x in ("vp", "fp", "fn")) if c == "total"
                      else (s[(c, "vp")], s[(c, "fp")], s[(c, "fn")]))
        p, r, f = prf(vp, fp, fn)
        res["objetos"][c] = {"vp": vp, "fp": fp, "fn": fn, "precisao": p, "recall": r, "f1": f}
        linhas_md.append([f"**{c}**" if c == "total" else c, vp + fn, vp, fp, fn, pct(p), pct(r), pct(f)])
    md.append("## Objetos (YOLO), por classe (IoU ≥ 0,5, mesma classe)\n")
    md.append(tabela_md(["Classe", "Marcados", "VP", "FP", "FN", "Precisão", "Recall", "F1"], linhas_md))
    linhas_md = []
    for c in sorted({c for (c, _) in cat["total"]}):
        s = cat["total"]
        p, r, f = prf(s[(c, "vp")], s[(c, "fp")], s[(c, "fn")])
        res["categorias"][c] = {"vp": s[(c, "vp")], "fp": s[(c, "fp")], "fn": s[(c, "fn")],
                                "precisao": p, "recall": r, "f1": f}
        linhas_md.append([c, s[(c, "vp")] + s[(c, "fn")], pct(p), pct(r), pct(f)])
    md.append("\nPor categoria (um carro detetado como camião conta como certo):\n")
    md.append(tabela_md(["Categoria", "Marcados", "Precisão", "Recall", "F1"], linhas_md))
    tot = classif["certas"] + classif["erradas"]
    res["classificacao"] = {"certas": classif["certas"], "erradas": classif["erradas"],
                            "acerto": classif["certas"] / tot if tot else None,
                            "trocas": {f"{a} -> {b}": v for (a, b), v in confusao_classes.most_common()}}
    md.append(f"\nClassificação: das {tot} caixas encontradas (IoU ≥ 0,5, qualquer classe), "
              f"{pct(res['classificacao']['acerto'])} com a classe certa"
              + (" (trocas: " + ", ".join(f"{a} → {b} ×{v}" for (a, b), v in confusao_classes.most_common(5)) + ")"
                 if confusao_classes else "") + ".\n")

    # linhas e estrada, total e por câmara
    linhas_md = []
    for chave in cams + ["total"]:
        s = lin[chave]
        p, r, f = prf(s["vp"], s["fp"], s["fn"])
        e = est[chave]
        pe, re_, _ = prf(e["vp"], e["fp"], e["fn"])
        iou = e["vp"] / (e["vp"] + e["fp"] + e["fn"]) if e["vp"] + e["fp"] + e["fn"] else None
        miou = float(np.mean(ious_est[chave])) if ious_est[chave] else None
        res["linhas"][chave] = {"vp": s["vp"], "fp": s["fp"], "fn": s["fn"], "precisao": p, "recall": r, "f1": f}
        res["estrada"][chave] = {"iou": iou, "iou_medio": miou, "precisao": pe, "recall": re_}
        nome = "**total**" if chave == "total" else f"câmara {chave}"
        linhas_md.append([nome, s["vp"] + s["fn"], pct(p), pct(r), pct(f), pct(iou), pct(miou), pct(pe), pct(re_)])
    md.append("\n## Linhas (CLRNet) e área circulável (YOLOP)\n")
    md.append(tabela_md(["", "Linhas marcadas", "Linhas: precisão", "recall", "F1",
                         "Estrada: IoU", "IoU médio", "precisão", "recall"], linhas_md))

    # faixa ao lado
    md.append("\n## Aviso de mudança de faixa (faixa ao lado, só com essa imagem)\n")
    estados_p = ["LIVRE", "OCUPADA", "SEM_FAIXA", "SEM_INFORMACAO"]
    total_f = Counter()
    for cam in cams:
        total_f.update(faixa[cam])
    if total_f:
        linhas_md = [[f"**{r}**"] + [total_f[(r, p)] for p in estados_p] for r in referencia.ESTADOS_FAIXA]
        md.append(tabela_md(["Marcado \\ Processamento"] + estados_p, linhas_md))
        certos = sum(total_f[(e, e)] for e in referencia.ESTADOS_FAIXA)
        n_f = sum(total_f.values())
        vp, fp, fn = (total_f[("OCUPADA", "OCUPADA")],
                      sum(v for (r, p), v in total_f.items() if p == "OCUPADA" and r != "OCUPADA"),
                      sum(v for (r, p), v in total_f.items() if r == "OCUPADA" and p != "OCUPADA"))
        p, r, f = prf(vp, fp, fn)
        res["faixa_ao_lado"] = {"acerto": certos / n_f, "n": n_f, "ocupada": {"precisao": p, "recall": r, "f1": f},
                                "matriz": {f"{a} -> {b}": v for (a, b), v in total_f.items()}}
        md.append(f"\nAcerto: {pct(certos / n_f)} ({certos} de {n_f}); OCUPADA: precisão {pct(p)}, recall {pct(r)}. "
                  "SEM_INFORMACAO = o processamento não viu o suficiente da faixa ao lado.\n")
    else:
        md.append("Nenhuma imagem com a faixa ao lado marcada (tecla F no anotar_referencia.py).\n")

    # figuras
    if args.figuras and not args.sem_imagens:
        destino = caminhos.RAIZ / "docs" / "img"
        destino.mkdir(parents=True, exist_ok=True)
        md.append("\n## Exemplos (original | referência | processamento)\n")
        for _, id_ in sorted(desacordo, reverse=True)[:args.figuras]:
            nome = f"referencia_{args.nome}_{id_}.png"
            shutil.copy(destino_img / f"{id_}_comparacao.png", destino / nome)
            md.append(f"![{id_}](../../img/{nome})\n")
            print(f"Figura: docs/img/{nome}")

    texto = "\n".join(md)
    (referencia.pasta(args.nome) / "metricas.md").write_text(texto, encoding="utf-8")
    (referencia.pasta(args.nome) / "metricas.json").write_text(json.dumps(res, indent=2, ensure_ascii=False),
                                                                encoding="utf-8")
    print(texto)
    print(f"Guardado em: {referencia.pasta(args.nome) / 'metricas.md'} (e metricas.json)"
          + ("" if args.sem_imagens else f"; imagens em {destino_img}"))


if __name__ == "__main__":
    main()
