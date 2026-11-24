# -*- coding: utf-8 -*-
"""
juntar_registos.py - Junta dois registos de uma câmara (feitos em sessões diferentes do
processar_bag.py, um com a câmara 5 e outro com a câmara 8, do mesmo bag) num registo de duas câmaras.

O registo novo fica igual ao que sairia de processar as duas câmaras juntas (deteccoes.mcap com um
tópico por câmara e ensaio.json com a lista "camaras"), por isso o reproduzir_registo.py e o
gui_resultados.py mostram as duas câmaras lado a lado e somam os veículos das duas.
Os registos originais não são alterados.

Trechos: os trechos das duas câmaras que se sobrepõem (ou se tocam) passam a ser um só trecho, do
início mais cedo ao fim mais tarde (a união). Cada trecho guarda em que intervalos cada câmara foi
registada ("intervalos_por_camara"); nas partes em que só uma câmara foi registada, a outra aparece
escura, com uma mensagem, e a soma dos veículos conta só a que existe.

  python offline/juntar_registos.py data/offline/<viagem>/camara5_<data> data/offline/<viagem>/camara8_<data>
"""
import argparse
import datetime
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "va_g05"))   # biblioteca comum (pacote va_g05)
from va_g05 import registo
from va_g05 import visual

parser = argparse.ArgumentParser(description="Junta dois registos de uma câmara (5 e 8, do mesmo bag) "
                                             "num registo de duas câmaras.")
parser.add_argument("ensaios", nargs=2, help="As duas pastas de ensaio (ou os seus ensaio.json)")
parser.add_argument("--pasta-registo", type=str, default=None,
                    help="Onde criar o registo juntado (por omissão: data/offline/<viagem>, ao lado dos originais)")
args = parser.parse_args()


def erro(msg):
    print(f"Erro: {msg}")
    sys.exit(1)


def tempo_s(t_ns, inicio_ns):
    return (t_ns - inicio_ns) / 1e9


# --- Ler os dois registos e confirmar que se podem juntar ---
fontes = []
for caminho in args.ensaios:
    pasta = registo.pasta_do_ensaio(caminho)
    e = registo.ler_ensaio(pasta)
    if e is None:
        erro(f"'{pasta}' não tem ensaio.json (não é uma pasta de registo do processar_bag.py).")
    cams = registo.camaras_do_ensaio(e)
    if len(cams) != 1:
        erro(f"'{pasta}' já tem {len(cams)} câmaras; só se juntam registos de uma câmara.")
    if not e.get("trechos"):
        erro(f"'{pasta}' não tem trechos.")
    fontes.append({"pasta": pasta, "ensaio": e, "info": cams[0], "camara": cams[0]["camara"]})

a, b = fontes
if a["camara"] == b["camara"]:
    erro(f"os dois registos são da mesma câmara ({a['camara']}).")
bag_a, bag_b = a["ensaio"]["bag"], b["ensaio"]["bag"]
if bag_a.get("nome") != bag_b.get("nome") or (
        bag_a.get("tamanho_bytes") and bag_b.get("tamanho_bytes") and bag_a["tamanho_bytes"] != bag_b["tamanho_bytes"]):
    erro(f"os registos não são do mesmo bag ({bag_a.get('nome')} e {bag_b.get('nome')}).")
fontes = sorted(fontes, key=lambda f: visual.ordenar_camaras([a["camara"], b["camara"]]).index(f["camara"]))
camaras = [f["camara"] for f in fontes]
inicio_bag = min(x for x in (bag_a.get("inicio_ns"), bag_b.get("inicio_ns")) if x) if (
    bag_a.get("inicio_ns") or bag_b.get("inicio_ns")) else None

if fontes[0]["ensaio"].get("modelos") != fontes[1]["ensaio"].get("modelos"):
    print("Aviso: os dois registos foram feitos com modelos ou limiares diferentes "
          "(ficam os dois no ensaio.json, em 'modelos_por_camara'; o resumo mostra os da câmara "
          f"{camaras[0]}).")

# --- União dos trechos ---
# cada trecho original: (início, fim, câmara); os que se sobrepõem ou tocam juntam-se num só
originais = sorted((t["inicio_ns"], t["fim_ns"], f["camara"]) for f in fontes for t in f["ensaio"]["trechos"])
uniao = []
for ini, fim, cam in originais:
    if uniao and ini <= uniao[-1]["fim_ns"]:
        uniao[-1]["fim_ns"] = max(uniao[-1]["fim_ns"], fim)
    else:
        uniao.append({"inicio_ns": ini, "fim_ns": fim, "intervalos_por_camara": {c: [] for c in camaras}})
    uniao[-1]["intervalos_por_camara"][cam].append([ini, fim])

# --- Deteções das duas câmaras ---
deteccoes = {}
for f in fontes:
    det = registo.ler_deteccoes(f["pasta"] / f["ensaio"].get("ficheiro_deteccoes", "deteccoes.mcap"))
    deteccoes[f["camara"]] = det.get(f["camara"]) or next(iter(det.values()), {})

# --- Escrever o registo juntado (mesmo formato do processar_bag.py com as duas câmaras) ---
viagem = fontes[0]["pasta"].parent
base = Path(args.pasta_registo) if args.pasta_registo else viagem
pasta_nova = base / f"camaras{'e'.join(camaras)}_juntado_{datetime.datetime.now():%Y%m%d_%H%M%S}"
proc = {f["camara"]: f["ensaio"].get("processamento", {}) for f in fontes}
n_proc = {c: p.get("imagens_processadas") or 0 for c, p in proc.items()}
tempos = {}
for k in ("objetos", "linhas", "estrada", "total"):
    pares = [(p.get("tempo_medio_ms", {}).get(k), n_proc[c]) for c, p in proc.items()]
    pares = [(v, n) for v, n in pares if v is not None]
    if pares:
        tempos[k] = round(sum(v * max(n, 1) for v, n in pares) / sum(max(n, 1) for _, n in pares), 1)
metadados = {
    "processado_com": "juntar_registos.py (dois registos de uma câmara do processar_bag.py)",
    "criado_em": datetime.datetime.now().isoformat(timespec="seconds"),
    "juntado_de": [{"camara": f["camara"], "pasta": str(f["pasta"]), "criado_em": f["ensaio"].get("criado_em"),
                    "trechos": [[t["inicio_ns"], t["fim_ns"]] for t in f["ensaio"]["trechos"]]} for f in fontes],
    "bag": {**fontes[0]["ensaio"]["bag"], "topico_imagem": None, "inicio_ns": inicio_bag},
    "camaras": [{"camara": f["camara"], "topico_imagem": f["info"].get("topico_imagem"),
                 "topico_deteccoes": registo.topico_deteccoes(f["camara"]),
                 "calibracao": f["info"].get("calibracao"), "imagem": f["info"].get("imagem")} for f in fontes],
    "calibracao": None,
    "modelos": fontes[0]["ensaio"].get("modelos", {}),
    "modelos_por_camara": {f["camara"]: f["ensaio"].get("modelos", {}) for f in fontes},
    "trechos_pedidos": None,
    # processado em duas sessões: a velocidade de cada uma está em processamento_por_camara
    "processamento": {"imagens_processadas": sum(n_proc.values()), "tempo_medio_ms": tempos,
                      "imagens_por_segundo": None, "com_interface": None},
    "processamento_por_camara": proc,
}
registador = registo.Registador(pasta_nova, camaras, metadados)
for n, t in enumerate(uniao, 1):
    registador.iniciar_trecho(extra={"intervalos_por_camara": t["intervalos_por_camara"]})
    mensagens = sorted((ts, c) for c in camaras for ts in deteccoes[c]
                       if t["inicio_ns"] <= ts <= t["fim_ns"])
    for ts, c in mensagens:
        dados = dict(deteccoes[c][ts])
        dados["trecho"] = n
        dados["camara"] = c
        registador.escrever(ts, dados, camara=c)
    registador.trecho["inicio_ns"], registador.trecho["fim_ns"] = t["inicio_ns"], t["fim_ns"]
    registador.terminar_trecho()
registador.fechar()

# --- Resumo ---
print(f"Registo juntado: {registador.pasta}")
print(f"  câmaras {registo.texto_camaras(camaras)}  (de {fontes[0]['pasta'].name} e {fontes[1]['pasta'].name})")
ref = inicio_bag or uniao[0]["inicio_ns"]
for t in registador.trechos:
    print(f"  trecho {t['n']}: {tempo_s(t['inicio_ns'], ref):.1f} s a {tempo_s(t['fim_ns'], ref):.1f} s  "
          f"({', '.join(f'câmara {c}: {n}' for c, n in t['imagens_por_camara'].items())} imagens)")
    dur = (t["fim_ns"] - t["inicio_ns"]) / 1e9
    for c in camaras:
        ints = t["intervalos_por_camara"][c]
        coberto = sum(b_ - a_ for a_, b_ in ints) / 1e9
        if not ints:
            print(f"      câmara {c}: não registada neste trecho (aparece escura)")
        elif coberto < dur - 0.05:
            txt = ", ".join(f"{tempo_s(a_, ref):.1f}–{tempo_s(b_, ref):.1f} s" for a_, b_ in ints)
            print(f"      câmara {c}: registada em {txt}; no resto do trecho ({dur - coberto:.1f} s) aparece escura")
print(f"Para ver: python offline/reproduzir_registo.py \"{registador.pasta}\"   (ou no gui_resultados.py)")
