import sys
from pathlib import Path
import cv2
import numpy as np
from mcap_ros2.reader import read_ros2_messages

# Se não passar caminho, procura automaticamente o .mcap mais recente na pasta /data
if len(sys.argv) < 2:
    mcap_files = sorted(Path("/data").rglob("*.mcap"))
    if not mcap_files:
        print("Erro: Nenhum ficheiro .mcap encontrado em /data.")
        sys.exit(1)
    mcap_file = mcap_files[-1]
else:
    mcap_file = Path(sys.argv[1])

print(f"A processar ROS Bag com OpenCV: {mcap_file}")

for msg in read_ros2_messages(str(mcap_file), topics=['/image_raw']):
    ros_msg = msg.ros_msg
    width, height = ros_msg.width, ros_msg.height

    if ros_msg.encoding in ['rgb8', 'bgr8']:
        frame = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape((height, width, 3))
        if ros_msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # --- AQUI PODE APLICAR O SEU TRATAMENTO OPENCV ---
        # Exemplo: mostrar o frame numa janela
        cv2.imshow("Leitura Direta do ROS Bag (.mcap)", frame)
        if cv2.waitKey(33) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
print("Processamento concluído.")
