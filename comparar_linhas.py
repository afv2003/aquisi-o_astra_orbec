# -*- coding: utf-8 -*-
"""
comparar_linhas.py - Comparação de dois modelos de deteção de linhas da estrada: CLRNet (o do
projeto, linhas.py) e UFLDv2 (ufld.py), no mesmo vídeo, lado a lado.

Segue o processamento offline (processar_bag.py): lê um bag (.mcap) de uma câmara, corrige a
distorção da lente com a calibração dessa câmara e passa a MESMA imagem aos dois modelos. O vídeo
aparece duas vezes, lado a lado: à esquerda com as linhas do CLRNet, à direita com as do UFLDv2.
Também aceita vídeos (.mp4, .avi, ...) e pastas de imagens, como o processar_video_teste.py (sem
correção da distorção, reduzidos a 640 px de largura).

Registo (como no processar_bag.py: tecla R, --trechos, --registar-tudo): as deteções de cada modelo
ficam em separado, como se fossem duas câmaras (comparacao.py), em data_comparacao/<origem>/<sessão>/.
Instantes: cada pausa (Espaço) guarda, numa pasta só desse instante, as imagens (original, a que os
modelos receberam, a de cada modelo e a comparação lado a lado, tal como no ecrã) e as deteções.
No fim (com janela), abre o resumo da comparação (resumo_comparacao.py).

  python comparar_linhas.py data/viagem_<data>/viagem_<data>_0.mcap --camara 5
  python comparar_linhas.py data/viagem_<data>/viagem_<data>_0.mcap --registar-tudo --sem-interface
  python comparar_linhas.py video.mp4
"""
import argparse
import datetime
import json
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
import comparacao
import leitura_bag
import linhas
import registo
import ufld
import visual
from comparacao import MODELOS, NOMES
from teclado import TecladoTerminal

EXTENSOES_VIDEO = {".mp4", ".avi", ".mov", ".mkv", ".m4v", ".webm"}
EXTENSOES_IMAGEM = {".jpg", ".jpeg", ".png", ".bmp"}

parser = argparse.ArgumentParser(
    description="Compara a deteção de linhas do CLRNet e do UFLDv2 no mesmo vídeo, lado a lado.")
parser.add_argument("entrada", type=str,
                    help="Bag (.mcap), vídeo (.mp4, .avi, ...) ou pasta com imagens (.jpg, .png)")
parser.add_argument("--camara", type=str, default=None,
                    help="Bag: câmara a usar (5 ou 8). Por omissão: a primeira do bag (a 5, num bag com as duas)")
parser.add_argument("--calib", type=str, default=None,
                    help="Bag: ficheiro de calibração (por omissão config/camera_astra_<N>.yaml)")
parser.add_argument("--sem-correcao", action="store_true", help="Bag: não corrigir a distorção da lente")
parser.add_argument("--conf-linhas", type=float, default=None,
                    help="CLRNet: confiança mínima (0 a 1) de uma linha. Por omissão: 0.4 (modelos/linhas.json)")
parser.add_argument("--corte-linhas", type=float, default=None,
                    help="CLRNet: fração da imagem ignorada em cima (0 a 1). Por omissão: 0.46 (modelos/linhas.json)")
parser.add_argument("--corte-ufld", type=float, default=None,
                    help="UFLDv2: fração da imagem ignorada em cima (0 a 1). Por omissão: 0.4, como no treino "
                         "(modelos/ufld.json)")
parser.add_argument("--fps", type=float, default=None,
                    help="Vídeo/pasta: imagens por segundo. Por omissão: as do vídeo; numa pasta, o tempo no "
                         "nome dos ficheiros ou 30")
parser.add_argument("--largura", type=int, default=640,
                    help="Vídeo/pasta: reduz as imagens a esta largura (por omissão 640, como as Astra; 0 = não reduz)")
parser.add_argument("--sem-interface", action="store_true",
                    help="Não mostrar o vídeo: o mais depressa possível; teclas R, Espaço e Q no terminal")
parser.add_argument("--trechos", type=str, default=None,
                    help="Registar estes trechos, em segundos desde o início, ex.: \"10-20,35.5-50,60-\"")
parser.add_argument("--registar-tudo", action="store_true", help="Registar o vídeo todo (o mesmo que --trechos 0-)")
parser.add_argument("--pasta-registo", type=str, default="data_comparacao",
                    help="Pasta das sessões (registo e instantes). Por omissão: data_comparacao")
parser.add_argument("--sem-resumo", action="store_true", help="Não abrir a janela do resumo no fim")
args = parser.parse_args()
if args.registar_tudo:
    args.trechos = "0-"
trechos_pedidos = None
if args.trechos:
    try:
        trechos_pedidos = registo.ler_trechos_texto(args.trechos)
    except ValueError as e:
        parser.error(f"--trechos: {e}")
for nome in ("conf_linhas", "corte_linhas", "corte_ufld"):
    valor = getattr(args, nome)
    if valor is not None and not 0 <= valor <= 1:
        parser.error(f"--{nome.replace('_', '-')} tem de estar entre 0 e 1 (recebeu {valor})")

entrada = Path(args.entrada)
if not entrada.exists():
    print(f"Erro: '{entrada}' não encontrado.")
    sys.exit(1)
e_bag = entrada.is_file() and entrada.suffix.lower() == ".mcap"


# --------------------------------------------------------------------------- #
# Entrada: bag (como no processar_bag.py) ou vídeo/pasta (como no processar_video_teste.py)
# Cada imagem: (imagem BGR original, instante em ns, segundos desde o início, número da imagem)
# --------------------------------------------------------------------------- #
def entrada_bag():
    erro = leitura_bag.verificar_mcap(entrada)
    if erro:
        print(f"Erro: {erro}")
        sys.exit(1)
    indice_ok = leitura_bag.bag_tem_indice(entrada)
    if not indice_ok:
        print("Aviso: o bag não tem o índice no fim; a ler sequencialmente, desde o início.")
    cams = leitura_bag.camaras_no_bag(leitura_bag.topicos_do_bag(entrada))
    if cams:
        ordem = visual.ordenar_camaras(cams)
        if args.camara and args.camara not in cams:
            print(f"Erro: indicou --camara {args.camara}, mas este bag só tem a(s) câmara(s) "
                  f"{registo.texto_camaras(ordem)}.")
            sys.exit(1)
        cam = args.camara or ordem[0]
        topicos = [cams[cam]]
        if not args.camara and len(cams) > 1:
            print(f"O bag tem as câmaras {registo.texto_camaras(ordem)}: a usar a {cam} "
                  f"(para a outra: --camara {ordem[1]}).")
    else:
        cam = args.camara or leitura_bag.camara_do_nome(entrada)
        topicos = ["/image_raw"]
    inicio_ns, fim_ns = leitura_bag.intervalo_do_bag(entrada) if indice_ok else (None, None)
    total = (fim_ns - inicio_ns) / 1e9 if inicio_ns else 0.0

    def gerar():
        n = 0
        for msg in leitura_bag.ler_mensagens(entrada, topicos, indice_ok):
            img = leitura_bag.imagem_bgr(msg.ros_msg)
            if img is None:
                continue
            yield img, msg.log_time_ns, msg.log_time_ns, n, msg.channel.topic
            n += 1
    return gerar(), cam, inicio_ns, total, {"tipo": "bag", "ficheiro": str(entrada.resolve()), "nome": entrada.name,
                                           "topico_imagem": topicos[0], "inicio_ns": inicio_ns,
                                           "duracao_s": round(total, 3) if total else None,
                                           "tamanho_bytes": entrada.stat().st_size}


def entrada_video():
    if entrada.is_dir():
        ficheiros = [f for f in entrada.iterdir() if f.suffix.lower() in EXTENSOES_IMAGEM]
        if not ficheiros:
            print(f"Erro: a pasta '{entrada}' não tem imagens ({', '.join(sorted(EXTENSOES_IMAGEM))}).")
            sys.exit(1)
        ficheiros.sort(key=lambda f: (int(f.stem) if f.stem.isdigit() else 0, f.name))
        if args.fps is None and all(f.stem.isdigit() and int(f.stem) > 10 ** 15 for f in ficheiros):
            t0 = int(ficheiros[0].stem)
            tempos = [(int(f.stem) - t0) / 1e9 for f in ficheiros]
            descricao = "pasta de imagens (tempo no nome dos ficheiros)"
        else:
            fps = args.fps or 30.0
            tempos = [i / fps for i in range(len(ficheiros))]
            descricao = f"pasta de imagens, {fps:g} imagens/s"

        def gerar():
            for n, (f, t) in enumerate(zip(ficheiros, tempos)):
                img = cv2.imread(str(f))
                if img is not None:
                    yield img, int(round(t * 1e9)), None, n, f.name
        return gerar(), None, 0, tempos[-1], {"tipo": "pasta", "ficheiro": str(entrada.resolve()),
                                              "nome": entrada.name, "descricao": descricao,
                                              "imagens": len(ficheiros)}
    if entrada.suffix.lower() not in EXTENSOES_VIDEO:
        print(f"Erro: '{entrada.name}' não é um bag (.mcap), um vídeo ({', '.join(sorted(EXTENSOES_VIDEO))}) "
              "nem uma pasta de imagens.")
        sys.exit(1)
    cap = cv2.VideoCapture(str(entrada))
    if not cap.isOpened():
        print(f"Erro: não foi possível abrir o vídeo '{entrada}'.")
        sys.exit(1)
    fps = args.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
    n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    def gerar():
        n = 0
        while True:
            ok, img = cap.read()
            if not ok:
                break
            yield img, int(round(n / fps * 1e9)), None, n, None
            n += 1
        cap.release()
    return gerar(), None, 0, (n_total - 1) / fps if n_total > 1 else 0.0, {
        "tipo": "video", "ficheiro": str(entrada.resolve()), "nome": entrada.name, "fps": fps,
        "imagens": n_total}


if e_bag:
    imagens, camara, inicio_ns, total_s, fonte = entrada_bag()
else:
    imagens, camara, inicio_ns, total_s, fonte = entrada_video()

# --- Correção da distorção (só nos bags; a calibração é das Astra) ---
corretor, origem_calib, calibracao = None, None, None
if e_bag and not args.sem_correcao:
    ficheiro = args.calib or (f"config/camera_astra_{camara}.yaml" if camara else "config/camera_astra.yaml")
    if Path(ficheiro).exists():
        calibracao, origem_calib = leitura_bag.carregar_calibracao_yaml(ficheiro), ficheiro
    else:
        calibracao, origem_calib = leitura_bag.carregar_calibracao_bag(entrada, camara), "/camera_info do bag"
    if calibracao is None:
        print(f"Aviso: sem calibração ('{ficheiro}' não existe e o bag não tem /camera_info); sem correção.")
    else:
        corretor = leitura_bag.CorretorDistorcao(calibracao)
        print(f"Câmara {camara}: correção da distorção com {origem_calib}")

# --- Os dois modelos ---
try:
    detetor = {"clrnet": linhas.DetetorLinhas(corte_topo=args.corte_linhas, confianca=args.conf_linhas)}
except FileNotFoundError as e:
    print(f"Erro: modelo do CLRNet não encontrado ({e}). Ver README, secção 5.1.")
    sys.exit(1)
try:
    detetor["ufld"] = ufld.DetetorUFLD(corte_topo=args.corte_ufld)
except FileNotFoundError:
    print("Erro: falta o modelo do UFLDv2 (modelos/ufld.onnx). Prepare-o uma vez com: python preparar_ufld.py")
    sys.exit(1)
cfg_clr, cfg_ufld = detetor["clrnet"].cfg, detetor["ufld"].cfg
descricao_modelo = {"clrnet": cfg_clr.get("descricao", "CLRNet"), "ufld": cfg_ufld.get("descricao", "UFLDv2")}
print(f"CLRNet: {descricao_modelo['clrnet']}  ·  {detetor['clrnet'].dispositivo}  ·  "
      f"corte do topo {cfg_clr['corte_topo']:.0%}  ·  confiança mínima {cfg_clr['conf_threshold']:g}")
print(f"UFLDv2: {descricao_modelo['ufld']}  ·  {detetor['ufld'].dispositivo}  ·  corte do topo {cfg_ufld['corte_topo']:.0%}")

# --- Sessão (registo e instantes) ---
if e_bag:
    origem = entrada.resolve().parent.name if entrada.resolve().parent.name.startswith(("viagem", "realtime")) \
        else entrada.stem
    nome_sessao = f"camara{camara or 'X'}_{datetime.datetime.now():%Y%m%d_%H%M%S}"
else:
    origem, nome_sessao = entrada.stem, f"{entrada.stem}_{datetime.datetime.now():%Y%m%d_%H%M%S}"
pasta_sessao = Path(args.pasta_registo) / origem / nome_sessao
base, k = pasta_sessao, 2
while pasta_sessao.exists():
    pasta_sessao = base.with_name(f"{base.name}_{k}")
    k += 1
metadados = {
    "tipo": "comparacao_linhas",
    "processado_com": "comparar_linhas.py",
    "criado_em": datetime.datetime.now().isoformat(timespec="seconds"),
    "camara": camara,
    "fonte": fonte,
    "calibracao": leitura_bag.calibracao_para_dict(calibracao, origem_calib) if calibracao is not None else None,
    "modelos": {
        "clrnet": {"nome": NOMES["clrnet"], "descricao": descricao_modelo["clrnet"],
                   "dispositivo": detetor["clrnet"].dispositivo, "confianca_minima": cfg_clr["conf_threshold"],
                   "corte_topo": cfg_clr["corte_topo"]},
        "ufld": {"nome": NOMES["ufld"], "descricao": descricao_modelo["ufld"],
                 "dispositivo": detetor["ufld"].dispositivo, "corte_topo": cfg_ufld["corte_topo"],
                 "criterio": "linha do meio com mais de 50% das âncoras; linha de fora com mais de 25%"},
    },
    "trechos_pedidos": args.trechos,
}
if e_bag:
    metadados["bag"] = fonte                     # o Registador calcula os segundos dos trechos a partir daqui
registador = comparacao.RegistadorComparacao(pasta_sessao, metadados, inicio_ns=inicio_ns or 0)

com_interface = not args.sem_interface
teclado = None
if not com_interface:
    teclado = TecladoTerminal()
    if not teclado.ativo:
        print("Aviso: a entrada do programa não é um terminal; as teclas não funcionam "
              "(use --trechos ou --registar-tudo para registar).")
print(f"Entrada: {fonte['nome']} ({fonte['tipo']})" + (f"  ·  câmara {camara}" if camara else "")
      + ("" if e_bag else "  ·  sem correção da distorção"
         + (f"  ·  reduzida a {args.largura} px de largura" if args.largura else "")))
print(f"Sessão: {pasta_sessao}")
print("\n--- CONTROLOS ---" + ("" if com_interface else " (teclas no terminal)"))
print("[Espaço] : Pausar (guarda o instante em .png, os dois modelos ao mesmo tempo) / Retomar")
if trechos_pedidos is None:
    print("[R]      : Começar / terminar um trecho do registo")
else:
    print(f"Registo automático dos trechos: {args.trechos} s")
if com_interface and corretor is not None:
    print("[C]      : Ligar / desligar a correção da distorção")
print("[Q]      : Terminar")

JANELA = "Comparacao de modelos de linhas"
if com_interface:
    cv2.namedWindow(JANELA, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

tempos = {m: None for m in MODELOS}
soma = {m: 0.0 for m in MODELOS}
n_proc = 0
corrigir = corretor is not None
janela_ajustada = False
n_instantes = 0
ultimo = None                                    # o que está no ecrã (para a pausa e o fim)


def media(m, ms):
    tempos[m] = ms if tempos[m] is None else 0.9 * tempos[m] + 0.1 * ms
    soma[m] += ms


def tempo_txt(seg):
    m_, s_ = divmod(max(0.0, seg), 60)
    return f"{int(m_):02d}:{s_:04.1f}"


def alternar_trecho():
    if registador.a_registar:
        t = registador.terminar_trecho()
        if t:
            print(f"\nRegisto: trecho {t['n']} terminado ({t['imagens_por_camara'].get('clrnet', 0)} imagens).")
    else:
        registador.iniciar_trecho()
        print(f"\nRegisto: trecho {registador.trecho['n']} iniciado.")


def compor(estado):
    """As duas vistas (cada uma com o seu painel) e a comparação lado a lado."""
    vistas = {}
    for m in MODELOS:
        img = estado["imagem_modelos"].copy()
        visual.desenhar_linhas(img, estado["linhas"][m])
        n = len(estado["linhas"][m])
        sub = f"{descricao_modelo[m].split(',')[0]}  ·  " + (f"câmara {camara}" if camara else fonte["nome"]) + "  ·  " + (
            "corrigida" if estado["corrigida"] else ("sem correção" if e_bag else "vídeo externo"))
        info = {"tempo": estado["t_rel"], "total": total_s, "subtitulo": sub,
                "cor_subtitulo": "#9CA3AF" if estado["corrigida"] or not e_bag else "#F59E0B",
                "modelos": [("linhas", NOMES[m], f"{n} linha" + ("" if n == 1 else "s"),
                             tempos[m], True, True)],
                "rodape": f"imagem {estado['n'] + 1}" + (f"  ·  REGISTO trecho {estado['trecho']}"
                                                          if estado["trecho"] else ""),
                "registo": (f"REGISTO  ·  trecho {estado['trecho']}" if estado["trecho"] and m == MODELOS[-1] else None)}
        vistas[m] = visual.compor(img, [], info)
    return vistas, visual.lado_a_lado([vistas[m] for m in MODELOS])


def guardar_instante(estado):
    """Pausa: guarda as imagens e as deteções deste instante numa pasta só dele."""
    global n_instantes
    n_instantes += 1
    mm, ss = divmod(max(0.0, estado["t_rel"]), 60)
    pasta = pasta_sessao / "instantes" / f"{n_instantes:03d}_{int(mm):02d}m{ss:05.2f}s"   # ex.: 001_01m12.37s
    pasta.mkdir(parents=True, exist_ok=True)
    vistas, juntas = compor(estado)
    cv2.imwrite(str(pasta / "original.png"), estado["original"])
    if estado["corrigida"]:
        cv2.imwrite(str(pasta / "corrigida.png"), estado["imagem_modelos"])
    for m in MODELOS:
        cv2.imwrite(str(pasta / f"{m}.png"), vistas[m])
    cv2.imwrite(str(pasta / "comparacao.png"), juntas)
    info = {
        "instante": {"stamp_ns": estado["stamp_ns"], "segundos": round(estado["t_rel"], 3),
                     "imagem": estado["n"], "trecho": estado["trecho"], "registado": estado["trecho"] is not None},
        "fonte": {k: v for k, v in fonte.items() if k in ("tipo", "ficheiro", "nome", "topico_imagem")},
        "camara": camara,
        "imagem_original": {"ficheiro": "original.png", "largura": estado["original"].shape[1],
                            "altura": estado["original"].shape[0],
                            "redimensionada_de": estado.get("tamanho_lido")},
        "imagem_dos_modelos": "corrigida.png" if estado["corrigida"] else "original.png",
        "nota": "as coordenadas das linhas são as da imagem_dos_modelos (a mesma imagem para os dois modelos)",
        "deteccoes": {m: estado["dados"][m] for m in MODELOS},
    }
    (pasta / "deteccoes.json").write_text(json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8")
    return pasta


t_msg_anterior = None
t_progresso = 0.0
saiu_cedo = False
paused = False
for original, stamp_ns, log_ns, n_img, ref in imagens:
    t_inicio = time.perf_counter()
    tamanho_lido = None
    if not e_bag and args.largura and original.shape[1] != args.largura:
        tamanho_lido = [original.shape[1], original.shape[0]]
        original = cv2.resize(original, (args.largura, round(original.shape[0] * args.largura / original.shape[1])),
                              interpolation=cv2.INTER_AREA)
    t_rel = (stamp_ns - (inicio_ns or 0)) / 1e9 if e_bag else stamp_ns / 1e9
    if e_bag and not inicio_ns:                  # bag sem índice: conta-se a partir da 1.ª imagem
        inicio_ns = stamp_ns
        t_rel = 0.0
        registador.inicio_ns = stamp_ns
        registador.metadados["bag"]["inicio_ns"] = stamp_ns
    if "imagem" not in registador.metadados:
        registador.metadados["imagem"] = {"largura": original.shape[1], "altura": original.shape[0]}

    # A MESMA imagem para os dois modelos (corrigida da distorção, se houver calibração)
    corrigida = corrigir and corretor is not None
    imagem_modelos = corretor(original) if corrigida else original
    linhas_det = {}
    for m in MODELOS:
        t0 = time.perf_counter()
        linhas_det[m] = detetor[m].detetar(imagem_modelos)
        media(m, (time.perf_counter() - t0) * 1000)
    n_proc += 1

    # Registo: as deteções de cada modelo, com o mesmo instante (como duas câmaras)
    if trechos_pedidos is not None:
        dentro = any(a <= t_rel <= b for a, b in trechos_pedidos)
        if dentro != registador.a_registar:
            alternar_trecho()
    trecho = registador.trecho["n"] if registador.a_registar else None
    dados = {}
    for m in MODELOS:
        d = registo.dados_deteccoes([], linhas_det[m], None, stamp_ns, trecho=trecho, corrigida=corrigida,
                                    modelos_ativos={"objetos": False, "linhas": True, "estrada": False},
                                    camara=camara, contornos=False)
        d["modelo"] = m
        d.pop("estrada_percentagem", None)
        dados[m] = d
        if registador.a_registar:
            registador.escrever(stamp_ns, d, camara=m)

    estado = {"original": original, "imagem_modelos": imagem_modelos, "linhas": linhas_det, "dados": dados,
              "stamp_ns": stamp_ns, "t_rel": t_rel, "n": n_img, "trecho": trecho, "corrigida": corrigida,
              "tamanho_lido": tamanho_lido}
    ultimo = estado

    # Ritmo da gravação
    t_s = stamp_ns / 1e9
    intervalo_ms = (t_s - t_msg_anterior) * 1000 if t_msg_anterior is not None else 33.0
    intervalo_ms = intervalo_ms if 0 < intervalo_ms < 1000 else 33.0
    t_msg_anterior = t_s

    if com_interface:
        _, juntas = compor(estado)
        if not janela_ajustada:
            cv2.resizeWindow(JANELA, juntas.shape[1], juntas.shape[0])
            janela_ajustada = True
        cv2.imshow(JANELA, juntas)
        espera = max(1, int(intervalo_ms - (time.perf_counter() - t_inicio) * 1000))
        key = cv2.waitKey(espera) & 0xFF
    else:
        if time.perf_counter() - t_progresso > 0.5:
            t_progresso = time.perf_counter()
            tot = f" / {tempo_txt(total_s)}" if total_s else ""
            ms = "  ".join(f"{NOMES[m]} {tempos[m]:.0f} ms" for m in MODELOS)
            reg = f"REGISTO trecho {trecho}" if trecho else "registo parado"
            print(f"\r{tempo_txt(t_rel)}{tot}  ·  {ms}  ·  {reg}".ljust(78), end="", flush=True)
        c = teclado.tecla() if teclado else None
        key = ord(c) if c else 255

    if key in (ord('q'), 27):
        saiu_cedo = True
        break
    elif key == ord(' '):
        paused = True
    elif key == ord('r') and trechos_pedidos is None:
        alternar_trecho()
    elif key == ord('c') and corretor is not None and com_interface:
        corrigir = not corrigir

    if paused:
        pasta = guardar_instante(estado)
        print(f"\nPausa em {tempo_txt(t_rel)}: instante guardado em {pasta}"
              + ("" if com_interface else " (Espaço para retomar, R trecho, Q terminar)"))
    while paused:
        if com_interface:
            kp = cv2.waitKey(0) & 0xFF
        else:
            c = teclado.tecla() if teclado else None
            kp = ord(c) if c else 255
            if c is None:
                time.sleep(0.05)
        if kp in (ord('q'), 27):
            saiu_cedo, paused = True, False
        elif kp == ord(' '):
            paused = False
        elif kp == ord('r') and trechos_pedidos is None:
            alternar_trecho()
    if saiu_cedo:
        break

if teclado:
    print()
    teclado.fechar()
if n_proc:
    print("\nTempo médio por imagem: " + " | ".join(f"{NOMES[m]} {soma[m] / n_proc:.0f} ms" for m in MODELOS)
          + f"  ({n_proc} imagens)")
registador.metadados["processamento"] = {
    "imagens_processadas": n_proc,
    "tempo_medio_ms": {m: round(soma[m] / n_proc, 1) for m in MODELOS} if n_proc else {},
    "com_interface": com_interface, "terminado_antes_do_fim": saiu_cedo, "instantes_guardados": n_instantes,
}
registador.fechar()
if registador.trechos:
    print(f"\nRegisto guardado em: {pasta_sessao}")
    for t in registador.trechos:
        print(f"  trecho {t['n']}: {t.get('inicio_s', 0):.1f} s a {t.get('fim_s', 0):.1f} s "
              f"({t['imagens_por_camara'].get('clrnet', 0)} imagens, os dois modelos)")
if n_instantes:
    print(f"Instantes guardados: {n_instantes}, em {pasta_sessao / 'instantes'}")
if registador.trechos or n_instantes:
    print(f"Resumo: python resumo_comparacao.py \"{pasta_sessao}\"")

if com_interface and not saiu_cedo and ultimo is not None:
    print("\nFim do vídeo alcançado!")
    cv2.imshow(JANELA, visual.ecra_fim(compor(ultimo)[1]))
    while (cv2.waitKey(0) & 0xFF) not in (ord('q'), 27):
        pass
if com_interface:
    cv2.destroyAllWindows()
    if not args.sem_resumo and (registador.trechos or n_instantes):
        import resumo_comparacao
        resumo_comparacao.abrir(pasta_sessao)
print("Comparação terminada.")
