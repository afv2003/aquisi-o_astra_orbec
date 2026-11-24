#!/usr/bin/env bash
# Desliga / volta a ligar a deteção da câmara da direita (a 8, com as duas câmaras) durante o online.
# Serve sobretudo sem janela (scripts/online.sh ... -i); com janela, use a tecla D. Corra noutro
# terminal, com o online a decorrer:
#   bash scripts/alternar_camara.sh
# Enquanto a deteção dessa câmara estiver desligada, o vídeo dela também não é gravado nos trechos
# (a outra câmara continua a ser processada e gravada, e mais vezes por segundo).
set -euo pipefail
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose() { docker compose -f "$RAIZ/compose.yaml" "$@"; }

if ! compose ps --status running percecao 2>/dev/null | grep -q percecao; then
    echo "Erro: o online não está a correr (lance-o com: bash scripts/online.sh 5 8 -i)."
    exit 1
fi
compose exec -T percecao bash -c \
    "source /opt/ros/jazzy/setup.bash && ros2 topic pub -w 1 --once /processamento/comando std_msgs/msg/String '{data: alternar_direita}' > /dev/null"
echo "Pedido enviado: a deteção da câmara da direita foi desligada ou voltou a ligar (veja o terminal do online)."
