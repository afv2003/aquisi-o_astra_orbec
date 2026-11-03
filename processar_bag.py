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

# Configurar argumentos da linha de comandos
parser = argparse.ArgumentParser(description="Processar ROS bag com visualização e deteção YOLO.")
parser.add_argument("mcap_file", type=str, help="Caminho para o ficheiro .mcap")
parser.add_argument("-v", action="store_true", help="Mostrar apenas vídeo (Desativa o YOLO)")
args = parser.parse_args()

mcap_file = Path(args.mcap_file)

if not mcap_file.exists():
    print(f"Erro: Ficheiro '{mcap_file}' não encontrado.")
    sys.exit(1)

# Inicializar o modelo YOLO se o utilizador não passou a flag -v
usar_yolo = not args.v
if usar_yolo:
    if YOLO is None:
        print("Erro: A biblioteca 'ultralytics' não está instalada. Corre: pip install ultralytics")
        sys.exit(1)
    print("A carregar modelo YOLOv8n (poderá demorar um pouco na primeira vez)...")
    # Usa o modelo mais leve e rápido (nano) para não encravar o vídeo
    modelo_yolo = YOLO('yolov8n.pt') 

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
print("[Q]      : Fechar o visualizador")

janela_nome = "Visualizador OpenCV - Leitura em Tempo Real"
cv2.namedWindow(janela_nome)

paused = False
saiu_cedo = False
ultimo_frame = None

# Lê as mensagens à medida que o vídeo avança
for msg in read_ros2_messages(str(mcap_file), topics=['/image_raw']):
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
        
    # --- PROCESSAMENTO YOLO ---
    if usar_yolo:
        # Executa a inferência de forma silenciosa para não inundar o terminal
        resultados = modelo_yolo(frame, verbose=False)
        # O método .plot() desenha automaticamente os retângulos e labels no frame
        frame = resultados[0].plot()
        
    ultimo_frame = frame.copy()

    # Calcula o tempo decorrido convertendo o log_time (datetime) para segundos
    if start_time_sec > 0:
        elapsed_sec = msg.log_time.timestamp() - start_time_sec
        texto_tempo = f"{elapsed_sec:.1f}s / {total_duration_sec:.1f}s"
        cv2.putText(frame, texto_tempo, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
    cv2.imshow(janela_nome, frame)
    
    key = cv2.waitKey(30) & 0xFF
    
    if key == ord('q') or key == 27:
        saiu_cedo = True
        break
    elif key == ord(' '):
        paused = True
        
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