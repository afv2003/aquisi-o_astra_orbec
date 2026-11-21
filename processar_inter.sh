#!/usr/bin/env bash
set -euo pipefail

CAMARAS_VALIDAS="5 8"
read -r -a DEVS <<< "${DEVS_VIDEO:-/dev/video2 /dev/video4}"

uso() {
    echo "Uso: bash processar_inter.sh <câmara no ${DEVS[0]}> [<câmara no ${DEVS[1]}>] [-i] [--sem-correcao]"
    echo "  -i             : executa sem interface gráfica (em segundo plano / headless)"
    echo "  --sem-correcao : não corrige a distorção da lente (por omissão usa config/camera_astra_<N>.yaml)"
    echo "  ex.: bash processar_inter.sh 5 8"
    echo "  ex.: bash processar_inter.sh 5 8 -i"
    echo "  Para forçar CPU num PC com GPU: USAR_GPU=0 bash processar_inter.sh 5 8"
    echo "  Sem janela, para desligar/ligar a câmara da direita: bash alternar_camara.sh (noutro terminal)"
    exit 1
}

export SEM_INTERFACE=0
export SEM_CORRECAO=0
camaras=()

for arg in "$@"; do
    case "$arg" in
        -i|--sem-interface)
            export SEM_INTERFACE=1
            ;;
        --sem-correcao)
            export SEM_CORRECAO=1
            ;;
        -h|--help)
            uso
            ;;
        *)
            camaras+=("$arg")
            ;;
    esac
done

[ ${#camaras[@]} -ge 1 ] && [ ${#camaras[@]} -le 2 ] || uso
if [ ${#camaras[@]} -eq 2 ] && [ "${camaras[0]}" = "${camaras[1]}" ]; then
    echo "Erro: indicou a câmara ${camaras[0]} duas vezes."
    exit 1
fi

export DEV_CAMARA5=/dev/null DEV_CAMARA8=/dev/null
export CAMARAS_ATIVAS="${camaras[*]}"

servicos=()
i=0
for cam in "${camaras[@]}"; do
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

# --- Deteção automática de GPU NVIDIA (omissão: CPU; forçar CPU com USAR_GPU=0) ---
COMPOSE_FILES=(-f compose_realtime_inter.yaml)
export ONNX_PKG="onnxruntime"

if [ "${USAR_GPU:-auto}" != "0" ] \
    && command -v nvidia-smi >/dev/null 2>&1 \
    && nvidia-smi >/dev/null 2>&1 \
    && docker info 2>/dev/null | grep -qi nvidia; then
    CUDA_MAJOR=$(nvidia-smi 2>/dev/null | sed -n 's/.*CUDA Version: \([0-9]\+\).*/\1/p' | head -n1 || true)
    if [ -n "$CUDA_MAJOR" ] && [ "$CUDA_MAJOR" -ge 13 ]; then
        export ONNX_PKG="onnxruntime-gpu[cuda,cudnn]"
    else
        export ONNX_PKG="onnxruntime-gpu[cuda,cudnn]<1.27"
    fi
    COMPOSE_FILES+=(-f compose_realtime_gpu.yaml)
    echo "Hardware detetado: GPU NVIDIA (CUDA ${CUDA_MAJOR:-?}) -> ${ONNX_PKG}"
else
    if [ "${USAR_GPU:-auto}" = "0" ]; then
        echo "Hardware: CPU (forçado por USAR_GPU=0)"
    elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
        echo "Hardware: GPU NVIDIA encontrada, mas falta o NVIDIA Container Toolkit no Docker -> a usar CPU"
    else
        echo "Hardware detetado: CPU -> onnxruntime"
    fi
fi

if [ "$SEM_INTERFACE" = "1" ]; then
    echo "Modo: SEM interface gráfica (-i)"
else
    echo "Modo: COM interface gráfica"
fi

# Nome da sessão: o mesmo para a pasta do bag (gravador_bag) e para o resumo (processador)
export SESSAO="realtime_$(date +%Y%m%d_%H%M%S)"
PASTA_SESSAO="data_realtime_inter/$SESSAO"

echo "A iniciar processamento para a(s) câmara(s): $CAMARAS_ATIVAS"
echo "Gravação (vídeo original + deteções) e resumo em: $PASTA_SESSAO/"
echo

# --abort-on-container-exit: ao terminar o processamento (tecla Q), pára também a gravação, para o
# bag acabar ao mesmo tempo. O Ctrl+C pára tudo da mesma forma; o script continua depois (o "trap"
# impede que o Ctrl+C o termine), para mostrar o resumo.
trap 'true' INT
if [ ${#camaras[@]} -eq 2 ]; then
    docker compose "${COMPOSE_FILES[@]}" up --build --abort-on-container-exit || true
else
    docker compose "${COMPOSE_FILES[@]}" up --build --abort-on-container-exit \
        --no-deps "${servicos[@]}" processador gravador_bag || true
fi
trap - INT

echo
if [ ! -f "$PASTA_SESSAO/resumo_online.json" ]; then
    echo "Não foi gravado o resumo ($PASTA_SESSAO/resumo_online.json): o processamento não chegou a arrancar?"
    exit 1
fi
echo "Gravação e resumo em: $PASTA_SESSAO/"
if [ "$SEM_INTERFACE" = "1" ]; then
    echo "(sem interface: o resumo não é mostrado; está no resumo_online.json)"
else
    echo "A abrir o resumo (feche a janela para terminar)..."
    docker compose -f compose_realtime_inter.yaml run --rm --no-deps processador \
        python3 resumo_online.py "$PASTA_SESSAO" || true
fi
