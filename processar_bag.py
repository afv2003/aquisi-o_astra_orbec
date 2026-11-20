import sys
import time
import argparse
import datetime
from pathlib import Path
import cv2
import numpy as np

# Módulos locais (na mesma pasta deste script): os três modelos correm em ONNX via onnxruntime
sys.path.insert(0, str(Path(__file__).resolve().parent))
import objetos
import linhas
import estrada
import visual        # desenho dos resultados e do painel (cores fixas por modelo e por classe)
import leitura_bag   # leitura dos bags e correção da distorção (também usado pelo reproduzir_registo.py)
import registo       # registo das deteções (mesmo formato do processamento em tempo real)
from teclado import TecladoTerminal

# Classes do YOLO mostradas por omissão: só elementos de estrada (nomes do COCO, ver README, secção 5).
# Para mudar a lista de forma permanente, altere-a aqui; para uma execução, use --classes-objetos.
CLASSES_ESTRADA = ["person", "bicycle", "car", "motorcycle", "bus", "truck", "traffic light", "stop sign"]

# Configurar argumentos da linha de comandos
parser = argparse.ArgumentParser(
    description="Processar ROS bag: correção da distorção, deteção de objetos (YOLO), de linhas (CLRNet) e da estrada (YOLOP).")
parser.add_argument("mcap_file", type=str, help="Caminho para o ficheiro .mcap")
parser.add_argument("-v", action="store_true",
                    help="Mostrar apenas o vídeo: o mesmo que --sem-objetos --sem-linhas --sem-estrada")
parser.add_argument("--sem-objetos", action="store_true", help="Não detetar objetos (YOLO)")
parser.add_argument("--sem-linhas", action="store_true", help="Não detetar as linhas da estrada (CLRNet)")
parser.add_argument("--sem-estrada", action="store_true", help="Não detetar a estrada (YOLOP)")
parser.add_argument("--conf-objetos", type=float, default=0.25,
                    help="Confiança mínima (0 a 1) para mostrar um objeto. Por omissão: 0.25")
parser.add_argument("--classes-objetos", type=str, default=None,
                    help="Classes do YOLO a detetar, separadas por vírgulas (ex.: \"car,truck,traffic light\"). "
                         "Por omissão: as de estrada (CLASSES_ESTRADA, no início do script). "
                         "'todas' deteta as 80 classes do COCO")
parser.add_argument("--camara", type=str, default=None,
                    help="Câmara(s) a processar: 5, 8 ou \"5,8\" (ambas, lado a lado: 5 à esquerda, 8 à direita). "
                         "Por omissão: todas as câmaras do bag (nos bags antigos, de uma câmara, a do nome "
                         "do bag: viagem_camara5_...). Usa config/camera_astra_<N>.yaml de cada câmara")
parser.add_argument("--calib", type=str, default=None,
                    help="Ficheiro de calibração a usar (sobrepõe-se a config/camera_astra_<N>.yaml; "
                         "só com uma câmara)")
parser.add_argument("--sem-correcao", action="store_true",
                    help="Não corrigir a distorção da lente (usar a imagem original)")
parser.add_argument("--corte-linhas", type=float, default=None,
                    help="Fração da imagem ignorada em cima na deteção de linhas (0 a 1). "
                         "Por omissão: a do modelo (modelos/linhas.json)")
parser.add_argument("--conf-linhas", type=float, default=None,
                    help="Confiança mínima (0 a 1) para mostrar uma linha. Por omissão: 0.4 "
                         "(modelos/linhas.json). Valores mais baixos mostram mais linhas, e mais falsas")
parser.add_argument("--conf-estrada", type=float, default=None,
                    help="Confiança mínima (0.5 a 1) para um píxel ser estrada. Por omissão: a classe mais "
                         "provável (como no YOLOP). Valores mais altos marcam menos área")
parser.add_argument("--sem-interface", action="store_true",
                    help="Não mostrar o vídeo: processa o mais depressa possível e mostra o progresso no "
                         "terminal (as teclas R, Espaço e Q funcionam no terminal)")
parser.add_argument("--trechos", type=str, default=None,
                    help="Registar estes trechos, em segundos desde o início do bag, ex.: \"10-20,35.5-50,60-\" "
                         "(\"60-\" = do segundo 60 até ao fim). Sem esta opção, os trechos marcam-se com a tecla R")
parser.add_argument("--registar-tudo", action="store_true",
                    help="Registar o vídeo todo (o mesmo que --trechos 0-)")
parser.add_argument("--pasta-registo", type=str, default="data_offline",
                    help="Pasta onde os registos são guardados (por omissão: data_offline)")
args = parser.parse_args()
if args.registar_tudo:
    args.trechos = "0-"
trechos_pedidos = None
if args.trechos:
    try:
        trechos_pedidos = registo.ler_trechos_texto(args.trechos)
    except ValueError as e:
        parser.error(f"--trechos: {e}")
if args.v:
    args.sem_objetos = args.sem_linhas = args.sem_estrada = True
for nome in ("conf_objetos", "conf_linhas", "conf_estrada"):
    valor = getattr(args, nome)
    if valor is not None and not 0 <= valor <= 1:
        parser.error(f"--{nome.replace('_', '-')} tem de estar entre 0 e 1 (recebeu {valor})")

mcap_file = Path(args.mcap_file)
erro = leitura_bag.verificar_mcap(mcap_file)
if erro:
    print(f"Erro: {erro}")
    sys.exit(1)

# Um MCAP tem no fim um índice (resumo), escrito quando a gravação termina corretamente.
# Se faltar (gravação interrompida ou cópia incompleta), o bag é lido sequencialmente, desde o início.
indice_ok = leitura_bag.bag_tem_indice(mcap_file)
if not indice_ok:
    print("Aviso: o bag não tem o índice no fim (gravação interrompida ou cópia incompleta). "
          "A ler as imagens sequencialmente, desde o início.")

# --- CÂMARA(S) E TÓPICO(S) A PROCESSAR (rastreabilidade) ---
# Bags novos (duas câmaras): cada câmara tem o seu tópico, /camara5/image_raw e /camara8/image_raw;
# por omissão processam-se todas as câmaras do bag, lado a lado (5 à esquerda, 8 à direita).
# Bags antigos (uma câmara): tópico /image_raw; a câmara lê-se do nome da pasta (viagem_camara5_...)
# ou de --camara.
topicos = leitura_bag.topicos_do_bag(mcap_file)
cams_no_bag = leitura_bag.camaras_no_bag(topicos)
camara_nome = leitura_bag.camara_do_nome(mcap_file)
pedidas = None
if args.camara:
    texto = args.camara.strip().lower()
    pedidas = (None if texto in ("todas", "ambas")
               else list(dict.fromkeys(c.strip() for c in texto.replace(";", ",").replace(" ", ",").split(",") if c.strip())))

if cams_no_bag:  # bag novo: uma ou duas câmaras, cada uma no seu tópico
    lista = " e ".join(sorted(cams_no_bag))
    if pedidas:
        fora = [c for c in pedidas if c not in cams_no_bag]
        if fora:
            print(f"Erro: indicou --camara {args.camara}, mas este bag só tem a(s) câmara(s) {lista}.")
            sys.exit(1)
        camaras, origem_cam = visual.ordenar_camaras(pedidas), "--camara"
    else:
        camaras, origem_cam = visual.ordenar_camaras(cams_no_bag), "tópicos do bag"
    topico_da_camara = {cams_no_bag[c]: c for c in camaras}
else:  # bag antigo (uma câmara, tópico /image_raw) ou sem índice de tópicos
    if pedidas and len(pedidas) > 1:
        print("Erro: este bag não tem um tópico por câmara (/camara5/image_raw, /camara8/image_raw); "
              "só se pode processar uma câmara.")
        sys.exit(1)
    camara = pedidas[0] if pedidas else None
    if camara and camara_nome and camara != camara_nome:
        print(f"Erro: indicou --camara {camara}, mas o bag foi gravado pela câmara {camara_nome}.")
        sys.exit(1)
    camara = camara or camara_nome
    origem_cam = "nome do bag" if camara_nome else "--camara"
    if not camara and not args.calib and not args.sem_correcao:
        print("Erro: não sei que câmara gravou este bag (o nome não tem 'camara<N>').")
        print("Indique-a com --camara 5 ou --camara 8.")
        sys.exit(1)
    camaras = [camara]
    topico_da_camara = {t: camara for t in ["/image_raw"] + ([f"/camara{camara}/image_raw"] if camara and not topicos else [])}
topicos_leitura = list(topico_da_camara)
duas = len(camaras) > 1
if args.calib and duas:
    print("Erro: --calib só se usa com uma câmara (com duas, cada uma usa o seu config/camera_astra_<N>.yaml). "
          "Use --camara 5 ou --camara 8.")
    sys.exit(1)

if camaras != [None]:
    print(f"Câmara{'s' if duas else ''}: {registo.texto_camaras(camaras)} (de: {origem_cam}) | "
          f"tópico{'s' if duas else ''}: {', '.join(topicos_leitura)}"
          + ("  ·  lado a lado: " + " | ".join(f"câmara {c}" for c in camaras) if duas else ""))


# --- CALIBRAÇÃO DE CADA CÂMARA (correção da distorção da lente) ---
calibracao, origem_calib, corretor = {}, {}, {}
for cam in camaras:
    calibracao[cam] = None
    if args.sem_correcao:
        continue
    ficheiro = args.calib or (f"config/camera_astra_{cam}.yaml" if cam else "config/camera_astra.yaml")
    if Path(ficheiro).exists():
        calibracao[cam], origem_calib[cam] = leitura_bag.carregar_calibracao_yaml(ficheiro), ficheiro
    else:
        calibracao[cam] = leitura_bag.carregar_calibracao_bag(mcap_file, cam)
        origem_calib[cam] = "/camera_info do bag"
    prefixo = f"Câmara {cam}: " if duas else ""
    if calibracao[cam] is None:
        print(f"Aviso: {prefixo}sem calibração ('{ficheiro}' não existe e o bag não tem /camera_info). "
              "A imagem NÃO será corrigida.")
    else:
        print(f"{prefixo}Calibração carregada de: {origem_calib[cam]}")
        corretor[cam] = leitura_bag.CorretorDistorcao(calibracao[cam])
com_calibracao = bool(corretor)

# --- MODELOS: cada um é carregado só se não tiver sido desligado na linha de comandos ---
# Objetos (YOLOv8n, modelos/objetos.onnx com o onnxruntime)
detetor_objetos = None
if args.sem_objetos:
    print(f"Deteção de objetos: DESLIGADA ({'-v' if args.v else '--sem-objetos'})")
else:
    pedido = args.classes_objetos
    if pedido is not None and pedido.strip().lower() == "todas":
        lista_classes = None
    else:
        lista_classes = CLASSES_ESTRADA if pedido is None else [c.strip() for c in pedido.split(",") if c.strip()]
    try:
        detetor_objetos = objetos.DetetorObjetos(confianca=args.conf_objetos, classes=lista_classes)
        print(f"Deteção de objetos: ATIVADA (YOLOv8n, {detetor_objetos.dispositivo}, "
              f"confiança mínima {args.conf_objetos:g})")
        print("  Classes: " + ("todas (80 do COCO)" if detetor_objetos.classes_ids is None
                               else ", ".join(detetor_objetos.nomes[i] for i in detetor_objetos.classes_ids)))
    except ValueError as e:
        print(f"Erro: {e}")
        sys.exit(1)
    except ImportError:
        print("Aviso: a biblioteca 'onnxruntime' não está instalada; a deteção de objetos fica DESLIGADA. "
              "Corre: pip install -r requirements.txt")
    except FileNotFoundError as e:
        print(f"Aviso: modelo de objetos não encontrado ({e}); a deteção de objetos fica DESLIGADA.")

# Linhas da estrada (CLRNet, modelos/linhas.onnx com o onnxruntime)
detetor_linhas = None
if args.sem_linhas:
    print(f"Deteção de linhas: DESLIGADA ({'-v' if args.v else '--sem-linhas'})")
else:
    try:
        detetor_linhas = linhas.DetetorLinhas(corte_topo=args.corte_linhas, confianca=args.conf_linhas)
        print(f"Deteção de linhas: ATIVADA ({detetor_linhas.cfg.get('descricao', 'CLRNet')}, "
              f"{detetor_linhas.dispositivo}, corte do topo {detetor_linhas.cfg['corte_topo']:.0%}, "
              f"confiança mínima {detetor_linhas.cfg['conf_threshold']:g})")
    except ImportError:
        print("Aviso: a biblioteca 'onnxruntime' não está instalada; a deteção de linhas fica DESLIGADA. "
              "Corre: pip install -r requirements.txt")
    except FileNotFoundError as e:
        print(f"Aviso: modelo de linhas não encontrado ({e}); a deteção de linhas fica DESLIGADA. "
              "Ver README, secção 4.")

# Estrada / área circulável (YOLOP, modelos/estrada.onnx com o onnxruntime)
detetor_estrada = None
if args.sem_estrada:
    print(f"Deteção da estrada: DESLIGADA ({'-v' if args.v else '--sem-estrada'})")
else:
    try:
        detetor_estrada = estrada.DetetorEstrada(confianca=args.conf_estrada)
        conf_txt = (f"confiança mínima {args.conf_estrada:g}" if args.conf_estrada is not None
                    else "classe mais provável")
        print(f"Deteção da estrada: ATIVADA (YOLOP, área circulável, {detetor_estrada.dispositivo}, {conf_txt})")
    except ImportError:
        print("Aviso: a biblioteca 'onnxruntime' não está instalada; a deteção da estrada fica DESLIGADA.")
    except FileNotFoundError as e:
        print(f"Aviso: modelo da estrada não encontrado ({e}); a deteção da estrada fica DESLIGADA. "
              "Ver README, secção 6.")

# Nome de cada modelo, como aparece no painel do vídeo
NOMES_LINHAS = {"clrnet_r34": "CLRNet", "clrnet_r50": "CLRNet", "clrernet_r34": "CLRerNet"}
nome_modelo = {
    "objetos": "YOLOv8n",
    "linhas": NOMES_LINHAS.get(detetor_linhas.cfg.get("modelo"), "CLRNet") if detetor_linhas else "CLRNet",
    "estrada": "YOLOP",
}

# Estado de cada deteção durante o vídeo (as teclas O, L, E e V ligam/desligam)
ativo = {"objetos": detetor_objetos is not None, "linhas": detetor_linhas is not None,
         "estrada": detetor_estrada is not None}
carregado = dict(ativo)

# 1. Obter o tempo total lendo o índice do .mcap
total_duration_sec = 0.0
start_time_sec = 0.0
start_time_ns = None
if indice_ok:
    start_time_ns, end_time_ns = leitura_bag.intervalo_do_bag(mcap_file)
    if start_time_ns is not None:
        total_duration_sec = (end_time_ns - start_time_ns) / 1e9
        start_time_sec = start_time_ns / 1e9

mins, secs = divmod(total_duration_sec, 60)
print(f"A ler ROS bag sequencialmente: {mcap_file.name}")
print(f"Tempo total do vídeo: {int(mins)}m {secs:.1f}s" if total_duration_sec > 0
      else "Tempo total do vídeo: desconhecido (o bag não tem índice)")

# --- REGISTO DAS DETEÇÕES ---
# Cada execução com registo é um "ensaio": uma pasta com um deteccoes.mcap (todas as imagens de
# todos os trechos registados) e um ensaio.json (bag de origem, câmara, calibração, modelos e trechos).
# A pasta só é criada quando começa o primeiro trecho.
nome_viagem = mcap_file.resolve().parent.name if mcap_file.resolve().parent.name.startswith("viagem") \
    else mcap_file.stem
nome_pasta = (f"camara{camaras[0] or 'X'}" if not duas else "camaras" + "e".join(camaras))
pasta_ensaio = Path(args.pasta_registo) / nome_viagem / f"{nome_pasta}_{datetime.datetime.now():%Y%m%d_%H%M%S}"
metadados = {
    "processado_com": "processar_bag.py (offline)",
    "criado_em": datetime.datetime.now().isoformat(timespec="seconds"),
    "bag": {"ficheiro": str(mcap_file.resolve()), "nome": mcap_file.name,
            "topico_imagem": None, "inicio_ns": start_time_ns,
            "duracao_s": round(total_duration_sec, 3) if total_duration_sec else None,
            "tamanho_bytes": mcap_file.stat().st_size},
    # uma entrada por câmara (pela ordem do ecrã); topico_imagem e imagem preenchem-se na 1.ª imagem
    "camaras": [{"camara": c, "topico_imagem": None, "topico_deteccoes": registo.topico_deteccoes(c),
                 "calibracao": leitura_bag.calibracao_para_dict(calibracao[c], origem_calib[c])
                 if calibracao[c] is not None else None,
                 "imagem": None} for c in camaras],
    "calibracao": (leitura_bag.calibracao_para_dict(calibracao[camaras[0]], origem_calib[camaras[0]])
                   if not duas and calibracao[camaras[0]] is not None else None),
    "modelos": {
        "objetos": {"nome": nome_modelo["objetos"], "carregado": carregado["objetos"],
                    "confianca_minima": args.conf_objetos,
                    "classes": ([detetor_objetos.nomes[i] for i in detetor_objetos.classes_ids]
                                if detetor_objetos.classes_ids is not None else "todas")
                               if detetor_objetos is not None else None},
        "linhas": {"nome": nome_modelo["linhas"], "carregado": carregado["linhas"],
                   "confianca_minima": detetor_linhas.cfg["conf_threshold"] if detetor_linhas else None,
                   "corte_topo": detetor_linhas.cfg["corte_topo"] if detetor_linhas else None},
        "estrada": {"nome": nome_modelo["estrada"], "carregado": carregado["estrada"],
                    "confianca_minima": args.conf_estrada},
    },
    "trechos_pedidos": args.trechos,
}
registador = registo.Registador(pasta_ensaio, camaras, metadados)
info_camara = {c["camara"]: c for c in registador.metadados["camaras"]}

com_interface = not args.sem_interface
teclado = None
if not com_interface:
    teclado = TecladoTerminal()
    if not teclado.ativo:
        print("Aviso: a entrada do programa não é um terminal; as teclas não funcionam "
              "(use --trechos ou --registar-tudo para registar).")

print("\n--- CONTROLOS ---" + ("" if com_interface else " (teclas no terminal)"))
print("[Espaço] : Pausar / Retomar")
if trechos_pedidos is None:
    print("[R]      : Começar / terminar um trecho do registo (pode haver vários)")
else:
    print(f"Registo automático dos trechos: {args.trechos} s (a tecla R não é usada)")
if com_interface:
    if com_calibracao:
        print("[C]      : Ligar / desligar a correção da distorção (para comparar)"
              + (" - nas duas câmaras" if duas else ""))
    if carregado["objetos"]:
        print("[O]      : Ligar / desligar a deteção de objetos (YOLO)")
    if carregado["linhas"]:
        print("[L]      : Ligar / desligar a deteção de linhas (CLRNet)")
    if carregado["estrada"]:
        print("[E]      : Ligar / desligar a deteção da estrada (YOLOP)")
    if any(carregado.values()):
        print("[V]      : Desligar todas as deteções (só vídeo) / voltar a ligá-las")
print("[Q]      : Terminar" + (" e fechar o visualizador" if com_interface else ""))
corrigir = com_calibracao

janela_nome = "Visualizador OpenCV - Leitura em Tempo Real"
if com_interface:
    # com duas câmaras a janela é larga: pode ser redimensionada (mantendo a proporção)
    cv2.namedWindow(janela_nome, (cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO) if duas else cv2.WINDOW_AUTOSIZE)

paused = False
saiu_cedo = False
ultimo_frame = None
# Por câmara: última imagem já desenhada (com o painel) e sem o painel (para o ecrã do fim),
# ritmo da gravação e número de imagens processadas. O ecrã junta as câmaras lado a lado.
ecra_cam = {c: None for c in camaras}
limpa_cam = {c: None for c in camaras}
t_ant_cam = {c: None for c in camaras}
ritmo_cam = {c: None for c in camaras}
n_img_cam = {c: 0 for c in camaras}
janela_ajustada = False


def juntar(imagens_cam):
    """Imagens das câmaras lado a lado (uma câmara ainda sem imagem aparece a escuro)."""
    ref = next((im for im in imagens_cam.values() if im is not None), None)
    if ref is None:
        return None
    h, w = ref.shape[:2]
    return visual.lado_a_lado([imagens_cam[c] if imagens_cam[c] is not None
                               else visual.imagem_espera(w, h, f"Câmara {c}: à espera da primeira imagem")
                               for c in camaras])

# Tempos de cada etapa (média móvel, em ms) e ritmo da gravação, para mostrar no vídeo
tempos = {"linhas": 0.0, "estrada": 0.0, "objetos": 0.0, "total": 0.0}
soma = {"linhas": 0.0, "estrada": 0.0, "objetos": 0.0, "total": 0.0}
contagem = {"linhas": 0, "estrada": 0, "objetos": 0, "total": 0}   # imagens em que cada etapa correu
t_msg_anterior = None
t_progresso = 0.0


def media(chave, valor_ms):
    tempos[chave] = valor_ms if contagem[chave] == 0 else 0.9 * tempos[chave] + 0.1 * valor_ms
    soma[chave] += valor_ms
    contagem[chave] += 1


def alternar_trecho():
    """Tecla R: começa um trecho novo ou termina o que está a ser registado."""
    if registador.a_registar:
        trecho = registador.terminar_trecho()
        if trecho:
            print(f"\nRegisto: trecho {trecho['n']} terminado ({trecho['imagens']} imagens).")
    else:
        registador.iniciar_trecho()
        print(f"\nRegisto: trecho {registador.trecho['n']} iniciado.")


def tempo_txt(seg):
    m_, s_ = divmod(max(0.0, seg), 60)
    return f"{int(m_):02d}:{s_:04.1f}"


for msg in leitura_bag.ler_mensagens(mcap_file, topicos_leitura, indice_ok):
    t_inicio = time.perf_counter()
    cam = topico_da_camara.get(msg.channel.topic, camaras[0])
    frame = leitura_bag.imagem_bgr(msg.ros_msg)
    if frame is None:
        continue
    height, width = frame.shape[:2]
    t_msg = msg.log_time.timestamp()
    if start_time_sec == 0:  # bag sem índice: conta-se a partir da primeira imagem
        start_time_sec = t_msg
    t_rel = t_msg - start_time_sec
    if info_camara[cam]["topico_imagem"] is None:
        info_camara[cam]["topico_imagem"] = msg.channel.topic
        info_camara[cam]["imagem"] = {"largura": width, "altura": height}
        if not duas:                      # campos de uma câmara, como nos ensaios antigos
            registador.metadados["bag"]["topico_imagem"] = msg.channel.topic
            registador.metadados["imagem"] = {"largura": width, "altura": height}
        if registador.metadados["bag"]["inicio_ns"] is None:
            registador.metadados["bag"]["inicio_ns"] = msg.log_time_ns
    n_img_cam[cam] += 1

    # --- CORREÇÃO DA DISTORÇÃO (antes das deteções, para trabalharem já na imagem corrigida) ---
    corrigida = corrigir and cam in corretor
    if corrigida:
        frame = corretor[cam](frame)

    # As três deteções correm sobre a imagem corrigida e ainda sem desenhos.
    # --- DETEÇÃO DE LINHAS (CLRNet) ---
    linhas_frame = []
    if ativo["linhas"]:
        t0 = time.perf_counter()
        linhas_frame = detetor_linhas.detetar(frame)
        media("linhas", (time.perf_counter() - t0) * 1000)

    # --- DETEÇÃO DA ESTRADA (YOLOP) ---
    mascara_estrada = None
    if ativo["estrada"]:
        t0 = time.perf_counter()
        mascara_estrada = detetor_estrada.detetar(frame)
        media("estrada", (time.perf_counter() - t0) * 1000)

    # --- DETEÇÃO DE OBJETOS (YOLO) ---
    caixas = []   # (x1, y1, x2, y2, classe, confiança)
    if ativo["objetos"]:
        t0 = time.perf_counter()
        caixas = detetor_objetos.detetar(frame)
        media("objetos", (time.perf_counter() - t0) * 1000)

    # --- REGISTO: trechos automáticos (--trechos) ou marcados com a tecla R (comuns às câmaras) ---
    if trechos_pedidos is not None:
        dentro = any(a <= t_rel <= b for a, b in trechos_pedidos)
        if dentro != registador.a_registar:
            alternar_trecho()
    if registador.a_registar:
        registador.escrever(msg.log_time_ns, registo.dados_deteccoes(
            caixas, linhas_frame, mascara_estrada, msg.log_time_ns, trecho=registador.trecho["n"],
            corrigida=bool(corrigida), modelos_ativos=dict(ativo), camara=cam), camara=cam)

    # Ritmo: intervalo entre imagens na gravação vs. tempo que o processamento demorou.
    # O ritmo da gravação de cada câmara é o intervalo entre as suas imagens; a espera usa o intervalo
    # até à imagem anterior de qualquer câmara (com duas câmaras, as imagens vêm intercaladas).
    intervalo_ms = (t_msg - t_msg_anterior) * 1000 if t_msg_anterior is not None else 33.0
    intervalo_ms = intervalo_ms if 0 <= intervalo_ms < 1000 else 33.0
    t_msg_anterior = t_msg
    int_cam = (t_msg - t_ant_cam[cam]) * 1000 if t_ant_cam[cam] is not None else 33.0
    int_cam = int_cam if 0 < int_cam < 1000 else 33.0
    t_ant_cam[cam] = t_msg
    ritmo_cam[cam] = int_cam if ritmo_cam[cam] is None else 0.95 * ritmo_cam[cam] + 0.05 * int_cam  # média, para não oscilar
    media("total", (time.perf_counter() - t_inicio) * 1000)
    img_s = 1000 / max(tempos["total"], 1e-3)
    texto_registo = None
    if registador.a_registar:
        inicio_trecho = registador.trecho["inicio_ns"]
        dur = (msg.log_time_ns - inicio_trecho) / 1e9 if inicio_trecho else 0.0
        texto_registo = f"REGISTO  ·  trecho {registador.trecho['n']}  ·  {tempo_txt(dur)}"

    if com_interface:
        # Desenho (cores fixas, ver visual.py): estrada por baixo, linhas por cima, objetos por cima de tudo
        visual.desenhar_estrada(frame, mascara_estrada)
        visual.desenhar_linhas(frame, linhas_frame)
        visual.desenhar_caixas(frame, caixas)
        limpa_cam[cam] = frame.copy()

        # Painel no canto superior esquerdo (tempo, câmara, um marcador por modelo, desempenho)
        texto_cam = f"Câmara {cam}" if cam else "Câmara não indicada"
        if cam not in corretor:
            subtitulo, cor_sub = f"{texto_cam}  ·  sem calibração", "#F59E0B"
        elif corrigida:
            subtitulo, cor_sub = f"{texto_cam}  ·  corrigida", "#9CA3AF"
        else:
            subtitulo, cor_sub = f"{texto_cam}  ·  original (sem correção)", "#F59E0B"
        n_obj, n_lin = len(caixas), len(linhas_frame)
        valores = {"objetos": f"{n_obj} objeto" + ("" if n_obj == 1 else "s"),
                   "linhas": f"{n_lin} linha" + ("" if n_lin == 1 else "s"),
                   "estrada": f"{mascara_estrada.mean():.0%} da imagem" if mascara_estrada is not None else ""}
        info = {"tempo": t_rel, "total": total_duration_sec,
                "subtitulo": subtitulo, "cor_subtitulo": cor_sub,
                "modelos": [(k, nome_modelo[k], valores[k], tempos[k], ativo[k], carregado[k])
                            for k in ("objetos", "linhas", "estrada")],
                "rodape": f"{img_s:.1f} img/s  ·  gravado a {1000 / ritmo_cam[cam]:.0f} img/s",
                # indicador de registo: um só, na câmara da direita
                "registo": texto_registo if cam == camaras[-1] or ecra_cam[camaras[-1]] is None else None}
        ecra_cam[cam] = visual.compor(frame, caixas, info)
        ecra = juntar(ecra_cam)
        if duas and not janela_ajustada:
            cv2.resizeWindow(janela_nome, ecra.shape[1], ecra.shape[0])
            janela_ajustada = True
        cv2.imshow(janela_nome, ecra)

        # Espera só o que falta para acompanhar o ritmo da gravação (nunca menos de 1 ms)
        espera = max(1, int(intervalo_ms - (time.perf_counter() - t_inicio) * 1000))
        key = cv2.waitKey(espera) & 0xFF
    else:
        # Sem janela: progresso no terminal (2 vezes por segundo) e teclas lidas do terminal
        if time.perf_counter() - t_progresso > 0.5:
            t_progresso = time.perf_counter()
            total_txt = f" / {tempo_txt(total_duration_sec)}" if total_duration_sec > 0 else ""
            estado = texto_registo or "registo parado"
            por_cam = ("  ·  " + "  ".join(f"c{c}: {n_img_cam[c]}" for c in camaras)) if duas else ""
            print(f"\r{tempo_txt(t_rel)}{total_txt}  ·  {img_s:.1f} img/s{por_cam}  ·  {estado}".ljust(78),
                  end="", flush=True)
        c = teclado.tecla() if teclado else None
        key = ord(c) if c else 255

    if key == ord('q') or key == 27:
        saiu_cedo = True
        break
    elif key == ord(' '):
        paused = True
    elif key == ord('r') and trechos_pedidos is None:
        alternar_trecho()
    elif key == ord('c') and com_calibracao and com_interface:
        corrigir = not corrigir
    elif key in (ord('o'), ord('l'), ord('e')) and com_interface:
        chave = {ord('o'): "objetos", ord('l'): "linhas", ord('e'): "estrada"}[key]
        if carregado[chave]:
            ativo[chave] = not ativo[chave]
    elif key == ord('v') and any(carregado.values()) and com_interface:
        ligar = not any(ativo.values())   # se alguma estiver ligada, desliga todas; senão, liga todas
        ativo = {k: ligar and carregado[k] for k in ativo}

    if paused and not com_interface:
        print("\nEm pausa (Espaço para retomar, R para começar/terminar um trecho, Q para terminar).")
    while paused:
        if com_interface:
            key_pause = cv2.waitKey(0) & 0xFF
        else:
            c = teclado.tecla() if teclado else None
            key_pause = ord(c) if c else 255
            if c is None:
                time.sleep(0.05)
        if key_pause == ord('q') or key_pause == 27:
            saiu_cedo = True
            paused = False
            break
        elif key_pause == ord(' '):
            paused = False
        elif key_pause == ord('r') and trechos_pedidos is None:
            alternar_trecho()

    if saiu_cedo:
        break

if not com_interface:
    print()
    if teclado:
        teclado.fechar()

if contagem["total"]:
    partes = [f"{k} {soma[k] / contagem[k]:.0f} ms" for k in ("objetos", "linhas", "estrada", "total")
              if contagem[k]]
    img_s_media = 1000 * contagem["total"] / max(soma["total"], 1e-3)
    print("\nTempo médio por imagem (nas imagens em que cada deteção esteve ligada): "
          + " | ".join(partes) + f" ({img_s_media:.1f} imagens/s)")
    if duas:
        print("Imagens processadas por câmara: " + ", ".join(f"câmara {c}: {n_img_cam[c]}" for c in camaras))

# Fecha o registo (o trecho em curso termina na última imagem processada); guarda também como correu
# o processamento (para o resumo do gui_resultados.py)
registador.metadados["processamento"] = {
    "imagens_processadas": contagem["total"],
    "imagens_por_camara": {str(c): n_img_cam[c] for c in camaras},
    "tempo_medio_ms": {k: round(soma[k] / contagem[k], 1) for k in ("objetos", "linhas", "estrada", "total")
                       if contagem[k]},
    "imagens_por_segundo": round(1000 * contagem["total"] / max(soma["total"], 1e-3), 2) if contagem["total"] else None,
    "com_interface": com_interface,
    "terminado_antes_do_fim": saiu_cedo,
}
registador.fechar()
if registador.trechos:
    n_img = sum(t["imagens"] for t in registador.trechos)
    pasta_ensaio = registador.pasta
    print(f"\nRegisto guardado em: {pasta_ensaio}")
    for t in registador.trechos:
        ini, fim = t.get("inicio_s"), t.get("fim_s")
        intervalo = f"{ini:.1f} s a {fim:.1f} s" if ini is not None else ""
        por_cam = (" - " + ", ".join(f"câmara {c}: {n}" for c, n in t.get("imagens_por_camara", {}).items())
                   if duas else "")
        print(f"  trecho {t['n']}: {intervalo} ({t['imagens']} imagens{por_cam})")
    print(f"  total: {len(registador.trechos)} trecho(s), {n_img} imagens")
    print(f"Para ver ou exportar um trecho: python reproduzir_registo.py \"{pasta_ensaio}\"")
    print("Para ver o resumo e o percurso: python gui_resultados.py")
elif trechos_pedidos is not None:
    print("\nNenhuma imagem dentro dos trechos pedidos; não foi criado registo.")

ultimo_frame = juntar(limpa_cam) if com_interface else None
if com_interface and not saiu_cedo and ultimo_frame is not None:
    print("\nFim do vídeo alcançado!")
    ultimo_frame = visual.ecra_fim(ultimo_frame)
    cv2.imshow(janela_nome, ultimo_frame)

    while True:
        if (cv2.waitKey(0) & 0xFF) in [ord('q'), 27]:
            break
elif not com_interface and not saiu_cedo:
    print("\nFim do vídeo alcançado!")

if com_interface:
    cv2.destroyAllWindows()
print("Processamento terminado.")
