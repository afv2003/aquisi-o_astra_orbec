import re
import sys
import argparse
from pathlib import Path
import cv2
import numpy as np
from mcap_ros2.reader import read_ros2_messages
from mcap.reader import make_reader

# Tentar importar o YOLO
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

# Deteção de linhas da estrada (CLRNet em ONNX): módulo linhas.py, na mesma pasta deste script
sys.path.insert(0, str(Path(__file__).resolve().parent))
import linhas

# Configurar argumentos da linha de comandos
parser = argparse.ArgumentParser(
    description="Processar ROS bag: correção da distorção, deteção de objetos (YOLO) e de linhas (CLRNet).")
parser.add_argument("mcap_file", type=str, help="Caminho para o ficheiro .mcap")
parser.add_argument("-v", action="store_true",
                    help="Mostrar apenas vídeo (desativa o YOLO e a deteção de linhas)")
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
args = parser.parse_args()

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

# Inicializar o modelo YOLO se o utilizador não passou a flag -v
usar_yolo = not args.v
if usar_yolo:
    if YOLO is None:
        print("Erro: A biblioteca 'ultralytics' não está instalada. Corre: pip install ultralytics")
        sys.exit(1)
    print("A carregar modelo YOLOv8n (poderá demorar um pouco na primeira vez)...")
    # Usa o modelo mais leve e rápido (nano) para não encravar o vídeo
    modelo_yolo = YOLO('yolov8n.pt') 

# Deteção de linhas: sempre ativa (exceto com -v). Usa modelos/linhas.onnx com o onnxruntime.
detetor_linhas = None
if not args.v:
    try:
        detetor_linhas = linhas.DetetorLinhas(corte_topo=args.corte_linhas)
        print(f"Deteção de linhas: ATIVADA ({detetor_linhas.cfg.get('descricao', 'CLRNet')}, "
              f"{detetor_linhas.dispositivo}, corte do topo {detetor_linhas.cfg['corte_topo']:.0%})")
    except ImportError:
        print("Aviso: a biblioteca 'onnxruntime' não está instalada; a deteção de linhas fica DESLIGADA. "
              "Corre: pip install -r requirements.txt")
    except FileNotFoundError as e:
        print(f"Aviso: modelo de linhas não encontrado ({e}); a deteção de linhas fica DESLIGADA. "
              "Ver README, secção 4.")

# 1. Obter o tempo total lendo o índice do .mcap
total_duration_sec = 0.0
start_time_sec = 0.0
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
print(f"Tempo total do vídeo: {int(mins)}m {secs:.1f}s")
print(f"Modo YOLO: {'ATIVADO' if usar_yolo else 'DESATIVADO'}")
print("\n--- CONTROLOS ---")
print("[Espaço] : Pausar / Retomar")
if calibracao is not None:
    print("[C]      : Ligar / desligar a correção da distorção (para comparar)")
print("[Q]      : Fechar o visualizador")
corrigir = calibracao is not None

janela_nome = "Visualizador OpenCV - Leitura em Tempo Real"
cv2.namedWindow(janela_nome)

paused = False
saiu_cedo = False
ultimo_frame = None

# Lê as mensagens à medida que o vídeo avança
for msg in read_ros2_messages(str(mcap_file), topics=topicos_leitura):
    ros_msg = msg.ros_msg
    width, height = ros_msg.width, ros_msg.height
    
    # Processa a cor
    if ros_msg.encoding == 'yuv422_yuy2':
        frame_yuv = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape((height, width, 2))
        frame = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV2BGR_YUYV)
    elif ros_msg.encoding in ['rgb8', 'bgr8']:
        frame = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape((height, width, 3))
        if ros_msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        continue

    # --- CORREÇÃO DA DISTORÇÃO (antes do YOLO, para trabalhar já na imagem corrigida) ---
    if corrigir:
        frame = corrigir_distorcao(frame)

    # --- DETEÇÃO DE LINHAS (sobre a imagem corrigida, antes de o YOLO desenhar por cima) ---
    linhas_frame = detetor_linhas.detetar(frame) if detetor_linhas is not None else []

    # --- PROCESSAMENTO YOLO ---
    if usar_yolo:
        # Executa a inferência de forma silenciosa para não inundar o terminal
        resultados = modelo_yolo(frame, verbose=False)
        # O método .plot() desenha automaticamente os retângulos e labels no frame
        frame = resultados[0].plot()

    if detetor_linhas is not None:
        linhas.desenhar_linhas(frame, linhas_frame)

    ultimo_frame = frame.copy()

    # Calcula o tempo decorrido convertendo o log_time (datetime) para segundos
    if start_time_sec > 0:
        elapsed_sec = msg.log_time.timestamp() - start_time_sec
        texto_tempo = f"{elapsed_sec:.1f}s / {total_duration_sec:.1f}s"
        cv2.putText(frame, texto_tempo, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    if calibracao is not None:
        cv2.putText(frame, ("CORRIGIDA" if corrigir else "ORIGINAL")
                    + (f"  (camara {camara})" if camara else ""), (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if corrigir else (0, 165, 255), 2)
    if detetor_linhas is not None:
        cv2.putText(frame, f"LINHAS: {len(linhas_frame)}", (20, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow(janela_nome, frame)
    
    key = cv2.waitKey(30) & 0xFF
    
    if key == ord('q') or key == 27:
        saiu_cedo = True
        break
    elif key == ord(' '):
        paused = True
    elif key == ord('c') and calibracao is not None:
        corrigir = not corrigir

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