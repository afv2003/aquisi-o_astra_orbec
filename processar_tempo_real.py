import os
import time
import threading
import json
import cv2
import numpy as np
from functools import partial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

import objetos
import linhas
import estrada
import visual

# Mesmas classes de estrada do processar_bag.py.
# Se quiseres detetar todas as 80 classes do COCO (como estava antes), muda para: CLASSES_ESTRADA = None
CLASSES_ESTRADA = ["person", "bicycle", "car", "motorcycle", "bus", "truck", "traffic light", "stop sign"]


class ProcessadorMultiCamara(Node):
    def __init__(self, camaras):
        super().__init__('processador_realtime_multi')

        self.ultimo_frame = {cam: None for cam in camaras}
        self.pubs_dados = {}
        self.pubs_imagem = {}

        for cam in camaras:
            # Subscritor dinâmico para cada câmara
            cb = partial(self.listener_callback, cam_id=cam)
            self.create_subscription(
                ROSImage, f'/camara{cam}/image_raw', cb, qos_profile_sensor_data)

            # Publicadores dinâmicos para cada câmara
            self.pubs_dados[cam] = self.create_publisher(
                String, f'/camara{cam}/processamento/dados_deteccoes', 10)
            self.pubs_imagem[cam] = self.create_publisher(
                ROSImage, f'/camara{cam}/processamento/imagem_anotada', 10)

    def listener_callback(self, msg, cam_id):
        self.ultimo_frame[cam_id] = msg

    def publicar_resultados(self, cam_id, caixas_yolo, linhas_pontos, mascara_estrada, frame_anotado, ros_msg_original):
        dados = {
            "objetos": [
                {"classe": c, "confianca": round(p, 2), "caixa": [int(x1), int(y1), int(x2), int(y2)]}
                for (x1, y1, x2, y2, c, p) in caixas_yolo
            ],
            "linhas": [
                [[float(pt[0]), float(pt[1])] for pt in linha]
                for linha in linhas_pontos
            ],
            "estrada_percentagem": float(mascara_estrada.mean()) if mascara_estrada is not None else 0.0
        }

        msg_string = String()
        msg_string.data = json.dumps(dados)
        self.pubs_dados[cam_id].publish(msg_string)

        msg_img = ROSImage()
        msg_img.header = ros_msg_original.header
        msg_img.height = frame_anotado.shape[0]
        msg_img.width = frame_anotado.shape[1]
        msg_img.encoding = 'bgr8'
        msg_img.is_bigendian = 0
        msg_img.step = frame_anotado.shape[1] * 3
        msg_img.data = frame_anotado.tobytes()
        self.pubs_imagem[cam_id].publish(msg_img)


def main():
    rclpy.init()

    # Lê as câmaras que o utilizador escolheu no terminal (ex: "5 8", "5", "8")
    camaras_str = os.environ.get('CAMARAS_ATIVAS', '5').strip()
    camaras_pedidas = camaras_str.split()
    sem_interface = os.environ.get('SEM_INTERFACE', '0') == '1'

    no_ros = ProcessadorMultiCamara(camaras_pedidas)

    thread_ros = threading.Thread(target=rclpy.spin, args=(no_ros,), daemon=True)
    thread_ros.start()

    detetor_objetos = objetos.DetetorObjetos(confianca=0.25, classes=CLASSES_ESTRADA)
    detetor_linhas = linhas.DetetorLinhas()
    detetor_estrada = estrada.DetetorEstrada()

    print(f"Objetos (YOLOv8n): {detetor_objetos.dispositivo}")
    print(f"Linhas  (CLRNet):  {detetor_linhas.dispositivo}")
    print(f"Estrada (YOLOP):   {detetor_estrada.dispositivo}")

    # Cria uma janela independente para cada câmara pedida (apenas se houver interface)
    if not sem_interface:
        for cam in camaras_pedidas:
            cv2.namedWindow(f"Tempo Real - Camara {cam}")

    modo_txt = "sem interface" if sem_interface else "com interface"
    print(f"A aguardar imagens da(s) câmara(s) {camaras_pedidas} ({modo_txt})...")

    try:
        while rclpy.ok():
            processou_frame = False

            # Percorre as câmaras ativas e processa caso haja um frame novo
            for cam in camaras_pedidas:
                msg = no_ros.ultimo_frame[cam]
                if msg is not None:
                    no_ros.ultimo_frame[cam] = None
                    processou_frame = True

                    width, height = msg.width, msg.height
                    if msg.encoding == 'yuv422_yuy2':
                        frame_yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 2))
                        frame = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV2BGR_YUYV)
                    elif msg.encoding in ['rgb8', 'bgr8']:
                        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3)).copy()
                        if msg.encoding == 'rgb8':
                            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    else:
                        continue

                    # Processamento (todos em ONNX)
                    linhas_frame = detetor_linhas.detetar(frame)
                    mascara_estrada = detetor_estrada.detetar(frame)
                    caixas = detetor_objetos.detetar(frame)

                    # Desenho
                    visual.desenhar_estrada(frame, mascara_estrada)
                    visual.desenhar_linhas(frame, linhas_frame)
                    visual.desenhar_caixas(frame, caixas)

                    info = {
                        "tempo": 0, "total": 0, "subtitulo": f"Camara {cam} - Tempo Real",
                        "cor_subtitulo": "#9CA3AF", "rodape": "A processar...",
                        "modelos": [
                            ("objetos", "YOLOv8n", f"{len(caixas)} objs", 0, True, True),
                            ("linhas", "CLRNet", f"{len(linhas_frame)} linhas", 0, True, True),
                            ("estrada", "YOLOP", f"{mascara_estrada.mean():.0%} da img" if mascara_estrada is not None else "0%", 0, True, True)
                        ]
                    }
                    frame = visual.compor(frame, caixas, info)

                    # Publicar resultados específicos da câmara atual
                    no_ros.publicar_resultados(cam, caixas, linhas_frame, mascara_estrada, frame, msg)

                    # Mostrar o frame na janela correspondente
                    if not sem_interface:
                        cv2.imshow(f"Tempo Real - Camara {cam}", frame)

            if not sem_interface:
                if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                    break
            elif not processou_frame:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if not sem_interface:
            cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()