#!/usr/bin/env bash
# Lançar a aquisição indicando que câmara está em cada /dev/video.
#
#   bash gravar.sh 5 8   ->  câmara 5 em /dev/video2, câmara 8 em /dev/video4
#   bash gravar.sh 8 5   ->  câmara 8 em /dev/video2, câmara 5 em /dev/video4
#   bash gravar.sh 5     ->  só a câmara 5, em /dev/video2
#
# A 1.ª câmara indicada fica no 1.º dispositivo, a 2.ª no 2.º.
# Para usar outros dispositivos: DEVS_VIDEO="/dev/video4 /dev/video6" bash gravar.sh 5 8
set -euo pipefail

CAMARAS_VALIDAS="5 8"
read -r -a DEVS <<< "${DEVS_VIDEO:-/dev/video2 /dev/video4}"

uso() {
    echo "Uso: bash gravar.sh <câmara no ${DEVS[0]}> [<câmara no ${DEVS[1]}>]"
    echo "  ex.: bash gravar.sh 5 8"
    exit 1
}

[ $# -ge 1 ] && [ $# -le 2 ] || uso
if [ $# -eq 2 ] && [ "$1" = "$2" ]; then
    echo "Erro: indicou a câmara $1 duas vezes."
    exit 1
fi

# o compose.yaml precisa das duas variáveis; a câmara não usada fica com /dev/null
export DEV_CAMARA5=/dev/null DEV_CAMARA8=/dev/null
servicos=()
i=0
for cam in "$@"; do
    case " $CAMARAS_VALIDAS " in
        *" $cam "*) ;;
        *) echo "Erro: câmara '$cam' desconhecida (use $CAMARAS_VALIDAS)."; exit 1 ;;
    esac
    dev="${DEVS[$i]}"
    if [ ! -e "$dev" ]; then
        echo "Erro: $dev não existe. A câmara $cam está ligada? Veja com: ls /dev/video*"
        exit 1
    fi
    export "DEV_CAMARA${cam}=${dev}"
    servicos+=("driver_camara${cam}")
    echo "Câmara ${cam}  ->  ${dev}   (tópico /camara${cam}/image_raw)"
    i=$((i + 1))
done

echo "Confirme no RViz: tape a lente de cada câmara e veja que imagem fica preta."
echo

if [ $# -eq 2 ]; then
    exec docker compose up
else
    exec docker compose up --no-deps "${servicos[@]}" rviz gravador_bag
fi
