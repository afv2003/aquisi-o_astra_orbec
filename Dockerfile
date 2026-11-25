# Imagem do G5 (ROS 2 Jazzy): drivers das câmaras, perceção, planeador, visualizador, RViz e Foxglove.
# Tudo o que o online precisa está aqui dentro (código, modelos ONNX, configuração e dependências);
# fora do contentor ficam só os dados (./data, ou o disco externo, montados em /data).
#   docker compose build                  (ONNX_PKG no .env: onnxruntime = CPU; GPU NVIDIA: README, 4.3)
# Quando a imagem base dos docentes estiver publicada no repositório comum:
#   docker compose build --build-arg BASE_IMAGE=<imagem-base-do-repositorio-comum>
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-c"]

ARG ONNX_PKG=onnxruntime

# ROS e sistema:
#  usb-cam + compressed-image-transport: driver das câmaras (imagem e JPEG, que é o que se grava)
#  rosbag2-storage-mcap: ros2 bag em MCAP (perfil gravar)
#  rviz2, foxglove-bridge: visualização (perfis rviz e foxglove)
#  python3-numpy, python3-pytest: o planeador comum (va_planeamento) corre com o python3 do sistema
#  python3-tk, libgl1, libglib2.0-0: janela do resumo (Tk) e OpenCV
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-usb-cam \
        ros-jazzy-compressed-image-transport \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-rviz2 \
        ros-jazzy-foxglove-bridge \
        python3-numpy \
        python3-pytest \
        python3-venv \
        python3-pip \
        python3-tk \
        libgl1 \
        libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Python do G5 num ambiente virtual (OpenCV, onnxruntime, mcap, ...), no PATH; vê também o ROS
# (PYTHONPATH do setup.bash). Ignora as linhas onnxruntime* do requirements.txt (usa ONNX_PKG), o
# tkintermapview (o mapa é só da janela de resultados do offline) e o matplotlib (só do tools/figura_bag.py)
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
COPY requirements.txt /tmp/
RUN grep -Ev '^[[:space:]]*(onnxruntime|tkintermapview|matplotlib)' /tmp/requirements.txt > /tmp/req_base.txt \
    && pip install --no-cache-dir -r /tmp/req_base.txt "${ONNX_PKG}" pytest

# Workspace ROS 2: o pacote comum (va_planeamento, sem alterações) e o do grupo (va_g05).
# O colcon corre com o python3 do sistema (o do ROS); os executáveis do va_g05 usam o do venv.
WORKDIR /ws
COPY src/ src/
RUN source /opt/ros/jazzy/setup.bash \
    && PATH="/usr/bin:$PATH" colcon build \
    && rm -rf build log
# Configuração e modelos lidos da pasta do pacote (não se copiam outra vez para install/)
ENV VA_G05_DIR=/ws/src/va_g05
# pytest (testes, CI): sem os plugins de testes do ROS (launch_testing), que precisam do Python do
# sistema (lark, ...); os testes do projeto não os usam
ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
# Os executáveis no PATH, para o compose os chamar diretamente: o "ros2 run" não passa o Ctrl+C
# (SIGINT do docker stop) ao nó, que assim não fecharia a gravação nem gravaria o resumo
ENV PATH="/ws/install/va_g05/lib/va_g05:/ws/install/va_planeamento/lib/va_planeamento:$PATH"

# Entrypoint: carrega o ROS 2 e o workspace antes de qualquer comando
RUN printf '%s\n' '#!/bin/bash' 'set -e' \
        'source /opt/ros/jazzy/setup.bash' \
        'if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi' 'exec "$@"' \
        > /entrypoint.sh && chmod +x /entrypoint.sh \
    && echo 'if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
