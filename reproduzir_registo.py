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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import registo
import visual
from reproducao import Reproducao, ErroRegisto

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

# --- Ler o ensaio, as deteções e encontrar o bag original (reproducao.py) ---
try:
    rep = Reproducao(args.ensaio, bag=args.bag)
except ErroRegisto as e:
    print(f"Erro: {e}")
    sys.exit(1)
pasta, ensaio, trechos = rep.pasta, rep.ensaio, rep.trechos
if args.trecho:
    pedidos = set(args.trecho)
    desconhecidos = pedidos - {t["n"] for t in trechos}
    if desconhecidos:
        print(f"Erro: trecho(s) {sorted(desconhecidos)} não existe(m). Trechos do registo:")
        descrever_trechos(ensaio)
        sys.exit(1)
    trechos = [t for t in trechos if t["n"] in pedidos]
print(f"Registo: {pasta}")
print(f"  {len(rep.deteccoes)} imagens com deteções; trechos a mostrar: {', '.join(str(t['n']) for t in trechos)}")
if rep.aviso_tamanho:
    print(f"Aviso: o bag '{rep.bag}' não tem o mesmo tamanho que o processado; pode não ser o mesmo ficheiro.")
print(f"  bag original: {rep.bag}")

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
    for t_ns, frame, dados in rep.imagens(t):
        t0 = time.perf_counter()
        if dados is None:
            sem_det += 1
        h, w = frame.shape[:2]
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
        intervalo_ms = (t_ns - t_anterior) / 1e6 if t_anterior else 33.0
        intervalo_ms = intervalo_ms if 0 < intervalo_ms < 1000 else 33.0
        t_anterior = t_ns
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
