import re
import sys
import time
import argparse
from pathlib import Path
import cv2
import numpy as np
from mcap_ros2.reader import read_ros2_messages
from mcap_ros2.decoder import DecoderFactory
from mcap.reader import make_reader, NonSeekingReader

# Tentar importar o YOLO
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

# Deteção de linhas da estrada (CLRNet em ONNX): módulo linhas.py, na mesma pasta deste script
sys.path.insert(0, str(Path(__file__).resolve().parent))
import linhas
import estrada

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
                    help="Confiança mínima (0 a 1) para mostrar um objeto. Por omissão: 0.25 (a do ultralytics)")
parser.add_argument("--classes-objetos", type=str, default=None,
                    help="Classes do YOLO a detetar, separadas por vírgulas (ex.: \"car,truck,traffic light\"). "
                         "Por omissão: as de estrada (CLASSES_ESTRADA, no início do script). "
                         "'todas' deteta as 80 classes do COCO")
parser.add_argument("--camara", type=str, default=None,
                    help="Câmara que gravou o bag (5 ou 8). Por omissão: lida do nome do bag "
                         "(viagem_camara5_...). Usa config/camera_astra_<N>.yaml")
parser.add_argument("--calib", type=str, default=None,
                    help="Ficheiro de calibração a usar (sobrepõe-se a --camara)")
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
args = parser.parse_args()
if args.v:
    args.sem_objetos = args.sem_linhas = args.sem_estrada = True
for nome in ("conf_objetos", "conf_linhas", "conf_estrada"):
    valor = getattr(args, nome)
    if valor is not None and not 0 <= valor <= 1:
        parser.error(f"--{nome.replace('_', '-')} tem de estar entre 0 e 1 (recebeu {valor})")

mcap_file = Path(args.mcap_file)

if not mcap_file.exists():
    print(f"Erro: Ficheiro '{mcap_file}' não encontrado.")
    sys.exit(1)

# Um ficheiro MCAP começa sempre com estes 8 bytes; se não, está vazio, incompleto ou não é MCAP
with open(mcap_file, "rb") as f:
    inicio = f.read(8)
if inicio != b"\x89MCAP0\r\n":
    tamanho = mcap_file.stat().st_size
    print(f"Erro: '{mcap_file}' não é um ficheiro MCAP válido ({tamanho} bytes).")
    if tamanho < 1024:
        print("O ficheiro está vazio: a gravação não gravou nada, ou a cópia falhou.")
    sys.exit(1)

# Um MCAP tem no fim um índice (resumo), escrito quando a gravação termina corretamente.
# Se a gravação foi interrompida (ou a cópia ficou incompleta), esse índice falta ou está
# estragado; nesse caso o bag é lido sequencialmente, desde o início, sem o índice.
def bag_tem_indice(caminho):
    try:
        with open(caminho, "rb") as f:
            make_reader(f).get_summary()
        return True
    except Exception:
        return False


def ler_mensagens(caminho, topicos):
    """
    Mensagens dos tópicos indicados, em ordem de tempo.
    * Bag com índice: os dados estão em blocos ("chunks"), cada um com um código de verificação
      (CRC). Um bloco estragado (pontos coloridos na imagem, ou erro de leitura) é saltado e a
      leitura continua no bloco seguinte.
    * Bag sem índice (cortado no fim): lido sequencialmente até ao ponto onde foi cortado.
    """
    lidas = 0
    if not indice_ok:
        with open(caminho, "rb") as f:
            leitor = NonSeekingReader(f, decoder_factories=[DecoderFactory()])
            try:
                for m in read_ros2_messages(leitor, topics=topicos, log_time_order=False):
                    lidas += 1
                    yield m
            except Exception as e:
                print(f"\nAviso: o bag está cortado ou com dados estragados a partir deste ponto "
                      f"({type(e).__name__}); foram lidas {lidas} mensagens.")
        return

    with open(caminho, "rb") as f:
        leitor = make_reader(f, validate_crcs=True, decoder_factories=[DecoderFactory()])
        resumo = leitor.get_summary()
        canais = {cid for cid, c in resumo.channels.items() if c.topic in topicos}
        blocos = sorted((c for c in resumo.chunk_indexes
                         if not c.message_index_offsets or canais & set(c.message_index_offsets)),
                        key=lambda c: (c.message_start_time, c.chunk_start_offset))
        inicio, ultimo = None, -1
        saltados, perdido_s = 0, 0.0
        while True:
            try:
                for m in read_ros2_messages(leitor, topics=topicos, start_time=inicio):
                    lidas += 1
                    ultimo = m.log_time_ns
                    yield m
                break
            except Exception as e:
                # o bloco estragado é o primeiro que ainda não foi lido; continua-se depois dele
                limite = max(ultimo, (inicio or 0) - 1)
                restantes = [c for c in blocos if c.message_end_time > limite]
                if not restantes:
                    print(f"\nAviso: dados estragados no fim do bag ({type(e).__name__}); "
                          f"foram lidas {lidas} mensagens.")
                    break
                mau = restantes[0]
                inicio = mau.message_end_time + 1
                saltados += 1
                perdido_s += max(0.0, (mau.message_end_time - max(mau.message_start_time, limite)) / 1e9)
                print(f"\nAviso: bloco estragado no bag ({type(e).__name__}); "
                      f"saltado (cerca de {perdido_s:.1f} s perdidos até agora, {saltados} bloco(s)).")
        if saltados:
            print(f"\nBag com {saltados} bloco(s) estragado(s), saltado(s): cerca de {perdido_s:.1f} s "
                  f"de vídeo em falta. O ficheiro foi danificado depois de gravado (cópia ou disco).")


indice_ok = bag_tem_indice(mcap_file)
if not indice_ok:
    print("Aviso: o bag não tem o índice no fim (gravação interrompida ou cópia incompleta). "
          "A ler as imagens sequencialmente, desde o início.")

# --- CÂMARA E TÓPICO A PROCESSAR (rastreabilidade) ---
# Bags novos (duas câmaras): cada câmara tem o seu tópico, /camara5/image_raw e /camara8/image_raw.
# Bags antigos (uma câmara): tópico /image_raw; a câmara lê-se do nome da pasta (viagem_camara5_...)
# ou de --camara.
def topicos_do_bag(caminho):
    try:
        with open(caminho, "rb") as f:
            resumo = make_reader(f).get_summary()
        return sorted({c.topic for c in resumo.channels.values()}) if resumo else []
    except Exception:
        return []


topicos = topicos_do_bag(mcap_file)
cams_no_bag = {}
for t in topicos:
    m = re.fullmatch(r"/camara([0-9A-Za-z]+)/image_raw", t)
    if m:
        cams_no_bag[m.group(1)] = t

encontradas = re.findall(r"camara([0-9A-Za-z]+?)(?=_|\.|$)", mcap_file.resolve().as_posix())
camara_nome = encontradas[-1] if encontradas else None

if cams_no_bag:  # bag novo: uma ou duas câmaras, cada uma no seu tópico
    lista = " e ".join(sorted(cams_no_bag))
    if args.camara:
        if args.camara not in cams_no_bag:
            print(f"Erro: indicou --camara {args.camara}, mas este bag só tem a(s) câmara(s) {lista}.")
            sys.exit(1)
        camara, origem_cam = args.camara, "--camara"
    elif len(cams_no_bag) == 1:
        camara, origem_cam = next(iter(cams_no_bag)), "tópico do bag"
    else:
        print(f"Este bag tem as câmaras {lista}. Indique qual quer processar, por exemplo:")
        print(f"  python processar_bag.py {args.mcap_file} --camara {sorted(cams_no_bag)[0]}")
        sys.exit(1)
    topicos_leitura = [cams_no_bag[camara]]
else:  # bag antigo (uma câmara, tópico /image_raw) ou sem índice de tópicos
    if args.camara and camara_nome and args.camara != camara_nome:
        print(f"Erro: indicou --camara {args.camara}, mas o bag foi gravado pela câmara {camara_nome}.")
        sys.exit(1)
    camara = args.camara or camara_nome
    origem_cam = "nome do bag" if camara_nome else "--camara"
    if not camara and not args.calib and not args.sem_correcao:
        print("Erro: não sei que câmara gravou este bag (o nome não tem 'camara<N>').")
        print("Indique-a com --camara 5 ou --camara 8.")
        sys.exit(1)
    topicos_leitura = ["/image_raw"] + ([f"/camara{camara}/image_raw"] if camara and not topicos else [])

if camara:
    print(f"Câmara: {camara} (de: {origem_cam}) | tópico: {', '.join(topicos_leitura)}")
if args.calib is None:
    args.calib = f"config/camera_astra_{camara}.yaml" if camara else "config/camera_astra.yaml"


# --- CALIBRAÇÃO DA CÂMARA (correção da distorção da lente) ---
def carregar_calibracao_yaml(caminho):
    """Lê um camera_astra.yaml (formato ROS camera_info) -> (K, D, nova_K, (largura, altura))."""
    import yaml
    with open(caminho, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    K = np.array(c["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"], dtype=np.float64)
    P = np.array(c["projection_matrix"]["data"], dtype=np.float64).reshape(3, 4)
    return K, D, P[:, :3], (int(c["image_width"]), int(c["image_height"]))


def carregar_calibracao_bag(caminho_mcap):
    """Alternativa: lê o 1.º /camera_info gravado no bag (se o usb_cam tiver calibração)."""
    try:
        for m in read_ros2_messages(str(caminho_mcap),
                                    topics=['/camera_info'] + ([f'/camara{camara}/camera_info'] if camara else [])):
            ci = m.ros_msg
            K = np.array(ci.k, dtype=np.float64).reshape(3, 3)
            if K[0, 0] == 0:  # usb_cam sem calibração publica zeros
                return None
            P = np.array(ci.p, dtype=np.float64).reshape(3, 4)
            return K, np.array(ci.d, dtype=np.float64), P[:, :3], (ci.width, ci.height)
    except Exception:
        pass
    return None


calibracao = None
if not args.sem_correcao:
    if Path(args.calib).exists():
        calibracao = carregar_calibracao_yaml(args.calib)
        origem_calib = args.calib
    else:
        calibracao = carregar_calibracao_bag(mcap_file)
        origem_calib = "/camera_info do bag"
    if calibracao is None:
        print(f"Aviso: sem calibração ('{args.calib}' não existe e o bag não tem /camera_info). "
              "A imagem NÃO será corrigida.")
    else:
        print(f"Calibração carregada de: {origem_calib}")

_mapas_correcao = {}


def corrigir_distorcao(img):
    """Remove a distorção da lente. Os mapas são calculados uma única vez por resolução."""
    h, w = img.shape[:2]
    if (w, h) not in _mapas_correcao:
        K, D, nova_K, (wc, hc) = calibracao
        if (w, h) != (wc, hc):
            print(f"Aviso: calibração feita em {wc}x{hc} mas o bag é {w}x{h}; a escalar K.")
            s = np.diag([w / wc, h / hc, 1.0])
            K, nova_K = s @ K, s @ nova_K
        _mapas_correcao[(w, h)] = cv2.initUndistortRectifyMap(K, D, None, nova_K, (w, h), cv2.CV_16SC2)
    m1, m2 = _mapas_correcao[(w, h)]
    return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)

# --- MODELOS: cada um é carregado só se não tiver sido desligado na linha de comandos ---
# Objetos (YOLOv8n, ultralytics/PyTorch)
modelo_yolo = None
if args.sem_objetos:
    print(f"Deteção de objetos: DESLIGADA ({'-v' if args.v else '--sem-objetos'})")
elif YOLO is None:
    print("Aviso: a biblioteca 'ultralytics' não está instalada; a deteção de objetos fica DESLIGADA. "
          "Corre: pip install -r requirements.txt")
else:
    print("A carregar modelo YOLOv8n (poderá demorar um pouco na primeira vez)...")
    modelo_yolo = YOLO('yolov8n.pt')   # versão "nano", a mais leve, para o vídeo não ficar lento
    # Classes a detetar: nomes -> números do modelo (o YOLO só procura estas)
    nomes_yolo = {nome: i for i, nome in modelo_yolo.names.items()}
    pedido = args.classes_objetos
    if pedido is not None and pedido.strip().lower() == "todas":
        classes_yolo = None
    else:
        lista = CLASSES_ESTRADA if pedido is None else [c.strip() for c in pedido.split(",") if c.strip()]
        desconhecidas = [c for c in lista if c not in nomes_yolo]
        if desconhecidas or not lista:
            print(f"Erro: classe(s) desconhecida(s) para o YOLO: {', '.join(desconhecidas) or '(nenhuma)'}.")
            print("Classes possíveis: " + ", ".join(modelo_yolo.names.values()))
            sys.exit(1)
        classes_yolo = sorted(nomes_yolo[c] for c in lista)
    try:
        import torch
        disp_yolo = 'GPU (' + torch.cuda.get_device_name(0) + ')' if torch.cuda.is_available() else 'CPU'
    except Exception:
        disp_yolo = "CPU"
    print(f"Deteção de objetos: ATIVADA (YOLOv8n, {disp_yolo}, confiança mínima {args.conf_objetos:g})")
    print("  Classes: " + ("todas (80 do COCO)" if classes_yolo is None
                           else ", ".join(modelo_yolo.names[i] for i in classes_yolo)))

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

# Estado de cada deteção durante o vídeo (as teclas O, L, E e V ligam/desligam)
ativo = {"objetos": modelo_yolo is not None, "linhas": detetor_linhas is not None,
         "estrada": detetor_estrada is not None}
carregado = dict(ativo)

# 1. Obter o tempo total lendo o índice do .mcap
total_duration_sec = 0.0
start_time_sec = 0.0
if indice_ok:
    try:
        with open(mcap_file, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            if summary and summary.statistics:
                start_time_ns = summary.statistics.message_start_time
                end_time_ns = summary.statistics.message_end_time
                total_duration_sec = (end_time_ns - start_time_ns) / 1e9
                start_time_sec = start_time_ns / 1e9
    except Exception as e:
        print(f"Aviso: Não foi possível calcular o tempo total: {e}")

mins, secs = divmod(total_duration_sec, 60)
print(f"A ler ROS bag sequencialmente: {mcap_file.name}")
print(f"Tempo total do vídeo: {int(mins)}m {secs:.1f}s" if total_duration_sec > 0
      else "Tempo total do vídeo: desconhecido (o bag não tem índice)")
print("\n--- CONTROLOS ---")
print("[Espaço] : Pausar / Retomar")
if calibracao is not None:
    print("[C]      : Ligar / desligar a correção da distorção (para comparar)")
if carregado["objetos"]:
    print("[O]      : Ligar / desligar a deteção de objetos (YOLO)")
if carregado["linhas"]:
    print("[L]      : Ligar / desligar a deteção de linhas (CLRNet)")
if carregado["estrada"]:
    print("[E]      : Ligar / desligar a deteção da estrada (YOLOP)")
if any(carregado.values()):
    print("[V]      : Desligar todas as deteções (só vídeo) / voltar a ligá-las")
print("[Q]      : Fechar o visualizador")
corrigir = calibracao is not None

janela_nome = "Visualizador OpenCV - Leitura em Tempo Real"
cv2.namedWindow(janela_nome)

paused = False
saiu_cedo = False
ultimo_frame = None

# Lê as mensagens à medida que o vídeo avança
# Tempos de cada etapa (média móvel, em ms) e ritmo da gravação, para mostrar no vídeo
tempos = {"linhas": 0.0, "estrada": 0.0, "objetos": 0.0, "total": 0.0}
soma = {"linhas": 0.0, "estrada": 0.0, "objetos": 0.0, "total": 0.0}
contagem = {"linhas": 0, "estrada": 0, "objetos": 0, "total": 0}   # imagens em que cada etapa correu
t_msg_anterior = None


def media(chave, valor_ms):
    tempos[chave] = valor_ms if contagem[chave] == 0 else 0.9 * tempos[chave] + 0.1 * valor_ms
    soma[chave] += valor_ms
    contagem[chave] += 1


def painel(img, textos):
    """Escreve os textos (texto, escala, cor) no canto superior esquerdo, sobre um fundo escuro
    semitransparente, para se lerem por cima da imagem e das caixas do YOLO."""
    fonte, margem = cv2.FONT_HERSHEY_SIMPLEX, 8
    medidas = [cv2.getTextSize(t, fonte, e, 2) for t, e, _ in textos]
    larg = max(w for (w, _), _ in medidas) + 2 * margem
    alt = sum(h + b + margem for (_, h), b in medidas) + margem
    fundo = img[0:alt, 0:larg]
    fundo[:] = (fundo * 0.35).astype(np.uint8)
    y = 0
    for (t, e, cor), ((_, h), b) in zip(textos, medidas):
        y += margem + h
        cv2.putText(img, t, (margem, y), fonte, e, cor, 2, cv2.LINE_AA)
        y += b


for msg in ler_mensagens(mcap_file, topicos_leitura):
    t_inicio = time.perf_counter()
    ros_msg = msg.ros_msg
    width, height = ros_msg.width, ros_msg.height
    
    # Processa a cor
    if ros_msg.encoding == 'yuv422_yuy2':
        frame_yuv = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape((height, width, 2))
        frame = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV2BGR_YUYV)
    elif ros_msg.encoding in ['rgb8', 'bgr8']:
        frame = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape((height, width, 3)).copy()
        if ros_msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        continue

    # --- CORREÇÃO DA DISTORÇÃO (antes do YOLO, para trabalhar já na imagem corrigida) ---
    if corrigir:
        frame = corrigir_distorcao(frame)

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
    n_objetos = 0
    if ativo["objetos"]:
        t0 = time.perf_counter()
        resultados = modelo_yolo(frame, conf=args.conf_objetos, classes=classes_yolo, verbose=False)
        media("objetos", (time.perf_counter() - t0) * 1000)
        n_objetos = len(resultados[0].boxes)
        frame = resultados[0].plot()   # cópia da imagem com as caixas e as etiquetas desenhadas

    # Desenho: estrada por baixo (semitransparente), linhas por cima
    if mascara_estrada is not None:
        estrada.desenhar_estrada(frame, mascara_estrada)
    linhas.desenhar_linhas(frame, linhas_frame)

    ultimo_frame = frame.copy()

    # Ritmo: intervalo entre imagens na gravação vs. tempo que o processamento demorou
    t_msg = msg.log_time.timestamp()
    intervalo_ms = (t_msg - t_msg_anterior) * 1000 if t_msg_anterior is not None else 33.0
    intervalo_ms = intervalo_ms if 0 < intervalo_ms < 1000 else 33.0
    t_msg_anterior = t_msg
    ritmo_ms = intervalo_ms if contagem["total"] == 0 else 0.95 * ritmo_ms + 0.05 * intervalo_ms  # média, para não oscilar
    media("total", (time.perf_counter() - t_inicio) * 1000)
    img_s = 1000 / max(tempos["total"], 1e-3)

    # Informação no canto superior esquerdo (tempo, correção, linhas, desempenho)
    if start_time_sec == 0:  # bag sem índice: conta-se a partir da primeira imagem
        start_time_sec = t_msg
    elapsed_sec = t_msg - start_time_sec
    textos = [(f"{elapsed_sec:.1f}s / {total_duration_sec:.1f}s" if total_duration_sec > 0
               else f"{elapsed_sec:.1f}s", 0.9, (0, 255, 0))]
    if calibracao is not None:
        textos.append((("CORRIGIDA" if corrigir else "ORIGINAL") + (f"  (camara {camara})" if camara else ""),
                       0.6, (0, 255, 0) if corrigir else (0, 165, 255)))
    estados = [("objetos", "OBJETOS", f"{n_objetos}", (255, 255, 255)),
               ("linhas", "LINHAS", f"{len(linhas_frame)}", (0, 255, 255)),
               ("estrada", "ESTRADA", f"{mascara_estrada.mean():.0%} da imagem" if mascara_estrada is not None
                else "", (0, 200, 0))]
    desempenho = []
    for chave, nome, valor, cor in estados:
        if not carregado[chave]:
            continue
        if ativo[chave]:
            textos.append((f"{nome}: {valor}", 0.6, cor))
            desempenho.append(f"{chave} {tempos[chave]:.0f} ms")
        else:
            desligado = {"objetos": "desligados", "linhas": "desligadas", "estrada": "desligada"}[chave]
            textos.append((f"{nome}: {desligado}", 0.6, (150, 150, 150)))
    desempenho.append(f"{img_s:.1f} img/s (gravado a {1000 / ritmo_ms:.0f})")
    textos.append((" | ".join(desempenho), 0.5, (255, 255, 255)))
    painel(frame, textos)

    cv2.imshow(janela_nome, frame)

    # Espera só o que falta para acompanhar o ritmo da gravação (nunca menos de 1 ms)
    espera = max(1, int(intervalo_ms - (time.perf_counter() - t_inicio) * 1000))
    key = cv2.waitKey(espera) & 0xFF
    
    if key == ord('q') or key == 27:
        saiu_cedo = True
        break
    elif key == ord(' '):
        paused = True
    elif key == ord('c') and calibracao is not None:
        corrigir = not corrigir
    elif key in (ord('o'), ord('l'), ord('e')):
        chave = {ord('o'): "objetos", ord('l'): "linhas", ord('e'): "estrada"}[key]
        if carregado[chave]:
            ativo[chave] = not ativo[chave]
    elif key == ord('v') and any(carregado.values()):
        ligar = not any(ativo.values())   # se alguma estiver ligada, desliga todas; senão, liga todas
        ativo = {k: ligar and carregado[k] for k in ativo}

    while paused:
        key_pause = cv2.waitKey(0) & 0xFF 
        if key_pause == ord('q') or key_pause == 27:
            saiu_cedo = True
            paused = False
            break
        elif key_pause == ord(' '):
            paused = False
            
    if saiu_cedo:
        break

if contagem["total"]:
    partes = [f"{k} {soma[k] / contagem[k]:.0f} ms" for k in ("objetos", "linhas", "estrada", "total")
              if contagem[k]]
    img_s_media = 1000 * contagem["total"] / max(soma["total"], 1e-3)
    print("\nTempo médio por imagem (nas imagens em que cada deteção esteve ligada): "
          + " | ".join(partes) + f" ({img_s_media:.1f} imagens/s)")

if not saiu_cedo and ultimo_frame is not None:
    print("\nFim do vídeo alcançado!")
    texto_fim = "FIM DO VIDEO"
    texto_tecla = "Prima Q para fechar"
    
    centro_x, centro_y = width // 2 - 150, height // 2
    
    cv2.putText(ultimo_frame, texto_fim, (centro_x, centro_y), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
    cv2.putText(ultimo_frame, texto_tecla, (centro_x - 30, centro_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    
    cv2.imshow(janela_nome, ultimo_frame)
    
    while True:
        if (cv2.waitKey(0) & 0xFF) in [ord('q'), 27]:
            break

cv2.destroyAllWindows()
print("Processamento terminado.")