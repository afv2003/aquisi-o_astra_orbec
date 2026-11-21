#!/usr/bin/env bash
# Desliga / volta a ligar a deteção da câmara da direita (a 8, com as duas câmaras) durante o
# processamento em tempo real. Serve sobretudo sem janela (processar_inter.sh ... -i); com janela,
# use a tecla D. Corra noutro terminal, na pasta do projeto, com o processamento a decorrer:
#   bash alternar_camara.sh
# Enquanto a deteção dessa câmara estiver desligada, o vídeo dela também não é gravado nos trechos
# (a outra câmara continua a ser processada e gravada, e mais vezes por segundo).
set -euo pipefail

if ! docker compose -f compose_realtime_inter.yaml ps --status running processador 2>/dev/null | grep -q processador; then
    echo "Erro: o processamento em tempo real não está a correr (lance-o com: bash processar_inter.sh 5 8 -i)."
    exit 1
fi
docker compose -f compose_realtime_inter.yaml exec -T processador bash -c \
    "source /opt/ros/jazzy/setup.bash && ros2 topic pub -w 1 --once /processamento/comando std_msgs/msg/String '{data: alternar_direita}' > /dev/null"
echo "Pedido enviado: a deteção da câmara da direita foi desligada ou voltou a ligar (veja o terminal do processamento)."
