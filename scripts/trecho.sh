#!/usr/bin/env bash
# Começa / termina um trecho durante o online ao vivo (como a tecla R na janela). Serve sobretudo sem
# janela (scripts/online.sh ... -i). Corra noutro terminal, com o online a decorrer:
#   bash scripts/trecho.sh
# A deteção corre sempre; o vídeo original e as deteções só são gravados dentro dos trechos.
set -euo pipefail
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose() { docker compose -f "$RAIZ/compose.yaml" "$@"; }

if ! compose ps --status running percecao 2>/dev/null | grep -q percecao; then
    echo "Erro: o online não está a correr (lance-o com: bash scripts/online.sh 5 8 -i)."
    exit 1
fi
compose exec -T percecao bash -c \
    "source /opt/ros/jazzy/setup.bash && ros2 topic pub -w 1 --once /processamento/comando std_msgs/msg/String '{data: trecho}' > /dev/null"
echo "Pedido enviado: o trecho começou ou terminou (veja o terminal do online)."
