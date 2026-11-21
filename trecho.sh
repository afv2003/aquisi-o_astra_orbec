#!/usr/bin/env bash
# Começa / termina um trecho durante o processamento em tempo real (como a tecla R na janela). Serve
# sobretudo sem janela (processar_inter.sh ... -i). Corra noutro terminal, na pasta do projeto, com o
# processamento a decorrer:
#   bash trecho.sh
# A deteção corre sempre; o vídeo original e as deteções só são gravados dentro dos trechos.
set -euo pipefail

if ! docker compose -f compose_realtime_inter.yaml ps --status running processador 2>/dev/null | grep -q processador; then
    echo "Erro: o processamento em tempo real não está a correr (lance-o com: bash processar_inter.sh 5 8 -i)."
    exit 1
fi
docker compose -f compose_realtime_inter.yaml exec -T processador bash -c \
    "source /opt/ros/jazzy/setup.bash && ros2 topic pub -w 1 --once /processamento/comando std_msgs/msg/String '{data: trecho}' > /dev/null"
echo "Pedido enviado: o trecho começou ou terminou (veja o terminal do processamento)."
