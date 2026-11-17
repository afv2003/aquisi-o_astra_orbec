#!/usr/bin/env bash
set -euo pipefail

CAMARAS_VALIDAS="5 8"
read -r -a DEVS <<< "${DEVS_VIDEO:-/dev/video2 /dev/video4}"

uso() {
    echo "Uso: bash processar_inter.sh <câmara no ${DEVS[0]}> [<câmara no ${DEVS[1]}>]"
    echo "  ex.: bash processar_inter.sh 5 8"
    exit 1
}

[ $# -ge 1 ] && [ $# -le 2 ] || uso
if [ $# -eq 2 ] && [ "$1" = "$2" ]; then
    echo "Erro: indicou a câmara $1 duas vezes."
    exit 1
fi

export DEV_CAMARA5=/dev/null DEV_CAMARA8=/dev/null
export CAMARAS_ATIVAS="$*" 

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
    echo "Câmara ${cam}  ->  ${dev}   (a captar para tempo real)"
    i=$((i + 1))
done

echo "A iniciar processamento para a(s) câmara(s): $CAMARAS_ATIVAS"
echo

if [ $# -eq 2 ]; then
    exec docker compose -f compose_realtime_inter.yaml up --build
else
    # CORREÇÃO: Adicionado o gravador_bag aqui
    exec docker compose -f compose_realtime_inter.yaml up --build --no-deps "${servicos[@]}" processador gravador_bag
fi
