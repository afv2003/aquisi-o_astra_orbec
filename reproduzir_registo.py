# -*- coding: utf-8 -*-
"""
reproduzir_registo.py - Refaz o vídeo de um registo de deteções (feito pelo processar_bag.py).

Junta as imagens do bag original (data/...) com as deteções guardadas no registo
(data_offline/<viagem>/<ensaio>/), imagem a imagem, pelo instante de cada imagem. Não volta a
correr os modelos: desenha o que foi registado. Serve para rever um trecho em particular, ou para
o exportar para um vídeo .mp4.

  python reproduzir_registo.py                              # lista os registos existentes
  python reproduzir_registo.py <pasta_do_ensaio>            # mostra todos os trechos, por ordem
  python reproduzir_registo.py <pasta_do_ensaio> --trecho 2 # só o trecho 2
  python reproduzir_registo.py <pasta_do_ensaio> --exportar # grava trecho_<N>.mp4 na pasta do ensaio
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import leitura_bag
import registo
import visual

parser = argparse.ArgumentParser(description="Refaz o vídeo de um registo de deteções do processar_bag.py.")
parser.add_argument("ensaio", nargs="?", default=None,
                    help="Pasta do ensaio (ou o seu ensaio.json / deteccoes.mcap). Sem nada: lista os registos")
parser.add_argument("--trecho", type=int, action="append", default=None,
                    help="Número do trecho a mostrar (pode repetir-se: --trecho 1 --trecho 3). Por omissão: todos")
parser.add_argument("--exportar", action="store_true",
                    help="Grava cada trecho num vídeo .mp4 (trecho_<N>.mp4, na pasta do ensaio)")
parser.add_argument("--sem-interface", action="store_true",
                    help="Não mostrar o vídeo (só com --exportar; é mais rápido)")
parser.add_argument("--bag", type=str, default=None,
                    help="Caminho do bag original, se tiver mudado de sítio desde o processamento")
parser.add_argument("--pasta-registo", type=str, default="data_offline",
                    help="Pasta dos registos, para a listagem (por omissão: data_offline)")
args = parser.parse_args()
if args.sem_interface and not args.exportar:
    parser.error("--sem-interface só faz sentido com --exportar")


def tempo_txt(seg):
    m, s = divmod(max(0.0, seg), 60)
    return f"{int(m):02d}:{s:04.1f}"


def descrever_trechos(ensaio):
    for t in ensaio.get("trechos", []):
        ini, fim = t.get("inicio_s"), t.get("fim_s")
        intervalo = f"{ini:7.1f} s a {fim:7.1f} s" if ini is not None else ""
        print(f"    trecho {t['n']}: {intervalo}  ({t['imagens']} imagens)")


# --- Sem ensaio indicado (ou uma pasta com vários): listar os registos existentes ---
if args.ensaio is not None and registo.ler_ensaio(args.ensaio) is None and Path(args.ensaio).is_dir():
    args.pasta_registo, args.ensaio = args.ensaio, None
if args.ensaio is None:
    pastas = registo.listar_ensaios(args.pasta_registo)
    if not pastas:
        print(f"Não há registos em '{args.pasta_registo}'. Faça um com o processar_bag.py "
              "(tecla R, --trechos ou --registar-tudo).")
        sys.exit(0)
    print(f"Registos em '{args.pasta_registo}':")
    for p in pastas:
        e = registo.ler_ensaio(p)
        print(f"\n  {p}")
        print(f"    bag: {e['bag']['nome']} | câmara {e.get('camara')} | criado em {e.get('criado_em')}")
        descrever_trechos(e)
    print("\nPara ver um registo: python reproduzir_registo.py <pasta> [--trecho N] [--exportar]")
    sys.exit(0)

# --- Ler o ensaio e as deteções ---
pasta = registo.pasta_do_ensaio(args.ensaio)
ensaio = registo.ler_ensaio(pasta)
if ensaio is None:
    print(f"Erro: '{pasta}' não tem ensaio.json (não é uma pasta de registo do processar_bag.py).")
    sys.exit(1)
trechos = ensaio.get("trechos", [])
if not trechos:
    print("Erro: o registo não tem trechos.")
    sys.exit(1)
if args.trecho:
    pedidos = set(args.trecho)
    desconhecidos = pedidos - {t["n"] for t in trechos}
    if desconhecidos:
        print(f"Erro: trecho(s) {sorted(desconhecidos)} não existe(m). Trechos do registo:")
        descrever_trechos(ensaio)
        sys.exit(1)
    trechos = [t for t in trechos if t["n"] in pedidos]

deteccoes = registo.ler_deteccoes(pasta / ensaio.get("ficheiro_deteccoes", "deteccoes.mcap"))
print(f"Registo: {pasta}")
print(f"  {len(deteccoes)} imagens com deteções; trechos a mostrar: {', '.join(str(t['n']) for t in trechos)}")

# --- Encontrar o bag original ---
candidatos = [Path(args.bag)] if args.bag else [
    Path(ensaio["bag"]["ficheiro"]),                                   # onde estava no processamento
    Path("data") / Path(ensaio["bag"]["ficheiro"]).parent.name / ensaio["bag"]["nome"],   # data/<viagem>/
]
bag = next((c for c in candidatos if c.exists()), None)
if bag is None:
    print("Erro: não encontrei o bag original. Procurei em:")
    for c in candidatos:
        print(f"  {c}")
    print("Indique onde está com --bag <caminho do .mcap>.")
    sys.exit(1)
erro = leitura_bag.verificar_mcap(bag)
if erro:
    print(f"Erro: {erro}")
    sys.exit(1)
tamanho = ensaio["bag"].get("tamanho_bytes")
if tamanho and bag.stat().st_size != tamanho:
    print(f"Aviso: o bag '{bag}' não tem o mesmo tamanho que o processado ({tamanho} bytes); "
          "pode não ser o mesmo ficheiro.")
print(f"  bag original: {bag}")
indice_ok = leitura_bag.bag_tem_indice(bag)
topico = ensaio["bag"]["topico_imagem"]
inicio_bag_ns = ensaio["bag"].get("inicio_ns") or min(t["inicio_ns"] for t in ensaio["trechos"])
duracao_bag = ensaio["bag"].get("duracao_s") or 0.0

corretor = None
if ensaio.get("calibracao"):
    corretor = leitura_bag.CorretorDistorcao(leitura_bag.calibracao_de_dict(ensaio["calibracao"]))
modelos = ensaio.get("modelos", {})
nome_modelo = {k: modelos.get(k, {}).get("nome", k) for k in ("objetos", "linhas", "estrada")}
carregado = {k: bool(modelos.get(k, {}).get("carregado", True)) for k in ("objetos", "linhas", "estrada")}
camara = ensaio.get("camara")

janela = "Reproducao do registo"
if not args.sem_interface:
    cv2.namedWindow(janela)
    print("\n--- CONTROLOS ---\n[Espaço] : Pausar / Retomar\n"
          "[1] [2] [3] : Velocidade: tempo real / 2x mais lento / 3x mais lento\n"
          "[-] [+]  : Mais lento / mais rápido (passos de 0.5x, entre 1x e 3x)\n"
          "[N]      : Passar ao trecho seguinte\n[Q]      : Sair\n"
          "(a velocidade só afeta a visualização; o vídeo exportado fica sempre em tempo real)")

# Abrandamento da visualização: 1 = tempo real, 3 = três vezes mais lento
abrandar = 1.0


def tecla_velocidade(tecla):
    """Muda o abrandamento conforme a tecla; devolve True se a tecla era de velocidade."""
    global abrandar
    if tecla in (ord('1'), ord('2'), ord('3')):
        abrandar = float(chr(tecla))
    elif tecla in (ord('-'), ord('_')):
        abrandar = min(3.0, abrandar + 0.5)
    elif tecla in (ord('+'), ord('=')):
        abrandar = max(1.0, abrandar - 0.5)
    else:
        return False
    return True

sair = False
for t in trechos:
    print(f"\nTrecho {t['n']}: {t.get('inicio_s', 0):.1f} s a {t.get('fim_s', 0):.1f} s ({t['imagens']} imagens)")
    escritor = None
    caminho_video = pasta / f"trecho_{t['n']}.mp4"
    if args.exportar:
        dur = (t["fim_ns"] - t["inicio_ns"]) / 1e9
        fps = (t["imagens"] - 1) / dur if dur > 0 and t["imagens"] > 1 else 30.0
    t_anterior = None
    n_img, sem_det = 0, 0
    for msg in leitura_bag.ler_mensagens(bag, [topico], indice_ok, t["inicio_ns"], t["fim_ns"]):
        t0 = time.perf_counter()
        frame = leitura_bag.imagem_bgr(msg.ros_msg)
        if frame is None:
            continue
        dados = deteccoes.get(msg.log_time_ns)
        if dados is None:
            sem_det += 1
        corrigida = bool(dados.get("corrigida", corretor is not None)) if dados else corretor is not None
        if corrigida and corretor is not None:
            frame = corretor(frame)
        h, w = frame.shape[:2]

        # Deteções registadas -> mesmos desenhos do processar_bag.py
        caixas, linhas_pts, mascara = [], [], None
        ativos = dict(carregado)
        if dados:
            ativos = {k: bool(v) for k, v in (dados.get("modelos_ativos") or carregado).items()}
            caixas = [(*o["caixa"], o["classe"], o["confianca"]) for o in dados.get("objetos", [])]
            linhas_pts = [np.array(l, dtype=np.float32) for l in dados.get("linhas", []) if len(l) > 1]
            if ativos.get("estrada") and dados.get("estrada_contornos") is not None:
                mascara = registo.mascara_dos_contornos(dados["estrada_contornos"], dados.get("estrada_buracos", []), w, h)
        visual.desenhar_estrada(frame, mascara)
        visual.desenhar_linhas(frame, linhas_pts)
        visual.desenhar_caixas(frame, caixas)

        n_obj, n_lin = len(caixas), len(linhas_pts)
        pct = dados.get("estrada_percentagem", 0.0) if dados else 0.0
        valores = {"objetos": f"{n_obj} objeto" + ("" if n_obj == 1 else "s"),
                   "linhas": f"{n_lin} linha" + ("" if n_lin == 1 else "s"),
                   "estrada": f"{pct:.0%} da imagem"}
        t_rel = (msg.log_time_ns - inicio_bag_ns) / 1e9
        info = {"tempo": t_rel, "total": duracao_bag,
                "subtitulo": (f"Registo  ·  câmara {camara}  ·  trecho {t['n']}"
                              + ("" if dados else "  ·  sem deteções")),
                "cor_subtitulo": "#9CA3AF" if dados else "#F59E0B",
                "modelos": [(k, nome_modelo[k], valores[k], None, ativos.get(k, False), carregado[k])
                            for k in ("objetos", "linhas", "estrada")],
                "rodape": f"{pasta.name}  ·  " + ("corrigida" if corrigida and corretor is not None else "original")}
        frame = visual.compor(frame, caixas, info)
        n_img += 1

        if args.exportar:
            if escritor is None:
                escritor = cv2.VideoWriter(str(caminho_video), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            escritor.write(frame)

        if args.sem_interface:
            if n_img % 15 == 0:
                print(f"\r  {n_img}/{t['imagens']} imagens", end="", flush=True)
            continue
        # indicador da velocidade (só na janela; o vídeo exportado já foi escrito sem ele)
        visual.etiqueta_canto(frame, "tempo real" if abrandar == 1 else f"{abrandar:g}x mais lento")
        cv2.imshow(janela, frame)
        intervalo_ms = (msg.log_time_ns - t_anterior) / 1e6 if t_anterior else 33.0
        intervalo_ms = intervalo_ms if 0 < intervalo_ms < 1000 else 33.0
        t_anterior = msg.log_time_ns
        espera = max(1, int(intervalo_ms * abrandar - (time.perf_counter() - t0) * 1000))
        tecla = cv2.waitKey(espera) & 0xFF
        tecla_velocidade(tecla)
        if tecla == ord(' '):
            while True:
                tecla = cv2.waitKey(0) & 0xFF
                tecla_velocidade(tecla)
                if tecla in (ord(' '), ord('q'), 27, ord('n')):
                    break
        if tecla in (ord('q'), 27):
            sair = True
            break
        if tecla == ord('n'):
            break

    if escritor is not None:
        escritor.release()
        print(f"\r  vídeo gravado: {caminho_video} ({n_img} imagens)".ljust(60))
    if sem_det:
        print(f"  Aviso: {sem_det} imagem(ns) do bag neste trecho não têm deteções no registo "
              "(por exemplo, blocos do bag que estavam estragados no processamento).")
    if sair:
        break

if not args.sem_interface:
    cv2.destroyAllWindows()
print("Terminado.")
