#!/usr/bin/env bash
# Online (Docker): ao vivo, com as câmaras, ou em reprodução de um bag, sem câmaras.
#
#   bash processar_inter.sh 5 8                 ao vivo: câmara 5 em /dev/video2, câmara 8 em /dev/video4
#   bash processar_inter.sh --bag <bag.mcap>    reprodução: as imagens vêm do bag (mesmo processamento)
#
# Serviços (compose_realtime_inter.yaml): drivers das câmaras (ou reprodutor), processador, planeador,
# visualizador (a janela) e, com --rviz, o RViz.
#
# Antes (README, secção 4.6): a imagem Docker construída (docker compose -f compose_realtime_inter.yaml build,
# com o ONNX_PKG certo para CPU ou GPU). Este script não a constrói. Corre-se na pasta do projeto (os
# caminhos, incluindo o do --bag, são relativos à pasta do terminal).
set -euo pipefail

CAMARAS_VALIDAS="5 8"
read -r -a DEVS <<< "${DEVS_VIDEO:-/dev/video2 /dev/video4}"

uso() {
    echo "Uso:"
    echo "  Ao vivo:     bash processar_inter.sh <câmara no ${DEVS[0]}> [<câmara no ${DEVS[1]}>] [opções]"
    echo "  Reprodução:  bash processar_inter.sh --bag <ficheiro .mcap> [opções]"
    echo "Opções:"
    echo "  -i                  : sem janela (sem o visualizador; corre só no terminal)"
    echo "  --rviz              : abre também o RViz (TF das câmaras, grelha local, caminho do A*, imagens)"
    echo "  --sem-correcao      : não corrige a distorção da lente (por omissão usa config/camera_astra_<N>.yaml)"
    echo "  ao vivo:"
    echo "  --registar-tudo     : começa logo um trecho (grava a sessão toda; a tecla R continua a funcionar)"
    echo "  --sem-processamento : só as câmaras e a gravação dos trechos (sem os modelos; para PCs mais fracos)"
    echo "  em reprodução:"
    echo "  --taxa <x>          : velocidade (1 = tempo real, 0.5 = metade, 2 = o dobro)"
    echo "  --camara <N>        : reproduzir só uma das câmaras do bag"
    echo "Antes: a imagem construída (docker compose -f compose_realtime_inter.yaml build; README, secção 4.6),"
    echo "  e o terminal na pasta do projeto."
    echo "Ao vivo, a deteção corre sempre; o vídeo original (JPEG) e as deteções só são gravados nos trechos:"
    echo "  tecla R na janela, ou, sem janela, bash trecho.sh (noutro terminal)."
    echo "  ex.: bash processar_inter.sh 5 8"
    echo "  ex.: bash processar_inter.sh 5 8 -i --registar-tudo"
    echo "  ex.: bash processar_inter.sh --bag data_realtime_inter/realtime_20261124_101500/realtime_20261124_101500_0.mcap"
    echo "  Para forçar CPU num PC com GPU: USAR_GPU=0 bash processar_inter.sh 5 8"
    echo "  Sem janela, para desligar/ligar a câmara da direita: bash alternar_camara.sh (noutro terminal)"
    exit 1
}

export SEM_INTERFACE=0 SEM_CORRECAO=0 REGISTAR_TUDO=0 SEM_PROCESSAMENTO=0 REPRODUCAO="" TAXA=1
RVIZ=0
BAG_PEDIDO=""
CAMARA_BAG=""
camaras=()

while [ $# -gt 0 ]; do
    case "$1" in
        -i|--sem-interface) SEM_INTERFACE=1 ;;
        --sem-correcao) SEM_CORRECAO=1 ;;
        --registar-tudo) REGISTAR_TUDO=1 ;;
        --sem-processamento) SEM_PROCESSAMENTO=1 ;;
        --rviz) RVIZ=1 ;;
        --bag) [ $# -ge 2 ] || uso; BAG_PEDIDO="$2"; shift ;;
        --taxa) [ $# -ge 2 ] || uso; TAXA="$2"; shift ;;
        --camara) [ $# -ge 2 ] || uso; CAMARA_BAG="$2"; shift ;;
        -h|--help) uso ;;
        -*) echo "Erro: opção desconhecida '$1'."; uso ;;
        *) camaras+=("$1") ;;
    esac
    shift
done

# --- Docker: CPU ou GPU NVIDIA (omissão: CPU; forçar CPU com USAR_GPU=0) ---
COMPOSE_FILES=(-f compose_realtime_inter.yaml)
export ONNX_PKG="onnxruntime"
if [ "$SEM_PROCESSAMENTO" = "1" ]; then
    echo "Hardware: sem processamento -> onnxruntime (CPU; os modelos não são usados)"
elif [ "${USAR_GPU:-auto}" != "0" ] \
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
    echo "  (a imagem tem de ter sido construída com: ONNX_PKG=\"${ONNX_PKG}\"; README, secção 4.6)"
else
    if [ "${USAR_GPU:-auto}" = "0" ]; then
        echo "Hardware: CPU (forçado por USAR_GPU=0)"
    elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
        echo "Hardware: GPU NVIDIA encontrada, mas falta o NVIDIA Container Toolkit no Docker -> a usar CPU"
    else
        echo "Hardware detetado: CPU -> onnxruntime"
    fi
fi
# o compose precisa destas variáveis em todos os modos (os serviços que não arrancam não as usam)
export DEV_CAMARA5=/dev/null DEV_CAMARA8=/dev/null BAG="" PASTA_BAG="$PWD" RELOGIO_NTP=""
compose() { docker compose "${COMPOSE_FILES[@]}" "$@"; }

servicos=()
if [ -n "$BAG_PEDIDO" ]; then
    # ------------------------------------------------------------ REPRODUÇÃO (sem câmaras)
    [ ${#camaras[@]} -eq 0 ] || { echo "Erro: com --bag não se indicam câmaras (use --camara <N> para escolher uma)."; exit 1; }
    [ "$SEM_PROCESSAMENTO" = "0" ] || { echo "Erro: --sem-processamento é só para o modo ao vivo."; exit 1; }
    [ "$REGISTAR_TUDO" = "0" ] || echo "Aviso: --registar-tudo não se usa em reprodução (nada é gravado)."
    if [ ! -f "$BAG_PEDIDO" ]; then
        echo "Erro: o ficheiro '$BAG_PEDIDO' não existe (indique o .mcap; pode estar no disco externo)."
        exit 1
    fi
    PASTA_BAG="$(cd "$(dirname "$BAG_PEDIDO")" && pwd)"
    BAG="$(basename "$BAG_PEDIDO")"
    export PASTA_BAG BAG
    export CAMARAS_ATIVAS="$CAMARA_BAG"
    # câmaras e trechos do bag (lidos dentro do Docker, como o reprodutor os vai ler)
    INFO=$(compose --profile reproduzir run --rm --no-deps -T reprodutor python3 reprodutor.py "/bag/$BAG" --info) || {
        echo "Erro: o bag não tem imagens de câmaras que se possam reproduzir."; exit 1; }
    CAMARAS_ATIVAS=$(printf '%s\n' "$INFO" | sed -n 's/^CAMARAS=//p')
    [ -n "$CAMARAS_ATIVAS" ] || { echo "Erro: nenhuma câmara no bag${CAMARA_BAG:+ (pedida: $CAMARA_BAG)}."; exit 1; }
    export CAMARAS_ATIVAS
    export REPRODUCAO="$BAG"
    export SESSAO="reproducao_$(basename "$BAG" .mcap)_$(date +%Y%m%d_%H%M%S)"
    servicos=(reprodutor processador planeador)
    echo "Modo: REPRODUÇÃO de $BAG_PEDIDO (câmara(s) $CAMARAS_ATIVAS; velocidade x$TAXA)"
    echo "  o processamento, o planeador e o visualizador são os mesmos do ao vivo; nada é gravado"
else
    # ------------------------------------------------------------ AO VIVO (câmaras)
    [ ${#camaras[@]} -ge 1 ] && [ ${#camaras[@]} -le 2 ] || uso
    if [ ${#camaras[@]} -eq 2 ] && [ "${camaras[0]}" = "${camaras[1]}" ]; then
        echo "Erro: indicou a câmara ${camaras[0]} duas vezes."
        exit 1
    fi
    export CAMARAS_ATIVAS="${camaras[*]}"
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
        echo "Câmara ${cam}  ->  ${dev}   (tópicos /camara${cam}/image_raw e /camara${cam}/image_raw/compressed)"
        i=$((i + 1))
    done
    servicos+=(processador)
    [ "$SEM_PROCESSAMENTO" = "1" ] || servicos+=(planeador)

    # --- Relógio do PC: os instantes das imagens vêm dele (o enunciado pede a hora certa, por NTP) ---
    if command -v timedatectl >/dev/null 2>&1; then
        RELOGIO_NTP=$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)
        NTP_LIGADO=$(timedatectl show -p NTP --value 2>/dev/null || true)
        if [ "$RELOGIO_NTP" = "yes" ]; then
            echo "Relógio do PC: sincronizado por NTP ($(date '+%Y-%m-%d %H:%M:%S %Z'))"
        else
            echo "ATENÇÃO: o relógio do PC NÃO está sincronizado por NTP (timedatectl: NTP=${NTP_LIGADO:-?})."
            echo "  Os instantes das imagens vêm deste relógio: os bags não ficam comparáveis com os de outros PCs."
            echo "  Com internet: sudo timedatectl set-ntp true ; espere um pouco e confirme com: timedatectl"
            if [ -t 0 ] && [ "${IGNORAR_RELOGIO:-0}" != "1" ]; then
                resposta=""
                read -r -p "  Continuar mesmo assim? [s/N] " resposta || true
                case "$resposta" in s|S|sim|Sim) ;; *) echo "Cancelado."; exit 1 ;; esac
            fi
        fi
    else
        echo "Aviso: sem timedatectl; o relógio é verificado só dentro do Docker (por NTP, se houver rede)."
    fi
    export RELOGIO_NTP
    export SESSAO="realtime_$(date +%Y%m%d_%H%M%S)"
    if [ "$SEM_PROCESSAMENTO" = "1" ]; then
        echo "Modo: SÓ GRAVAÇÃO (--sem-processamento): câmaras e trechos, sem os modelos"
    fi
    if [ "$REGISTAR_TUDO" = "1" ]; then
        echo "Gravação: desde o início (--registar-tudo)"
    else
        echo "Gravação: só nos trechos (nada é gravado até começar o primeiro)"
    fi
fi

if [ "$SEM_INTERFACE" = "1" ]; then
    echo "Ecrã: SEM janela (-i)  -  trechos: bash trecho.sh, noutro terminal"
else
    servicos+=(visualizador)
    echo "Ecrã: janela do visualizador (R trecho, D câmara da direita, Espaço pausa na reprodução, Q terminar)"
fi
PASTA_SESSAO="data_realtime_inter/$SESSAO"
if [ -n "$BAG_PEDIDO" ]; then
    echo "Resumo da reprodução em: $PASTA_SESSAO/"
else
    echo "Gravação (vídeo original em JPEG + deteções, só nos trechos) e resumo em: $PASTA_SESSAO/"
fi
echo

# O RViz arranca à parte: fechá-lo não termina a sessão
if [ "$RVIZ" = "1" ]; then
    compose --profile rviz up -d --no-deps rviz || echo "Aviso: o RViz não arrancou."
fi

# --abort-on-container-exit: ao terminar o processamento (tecla Q, fim do bag, Ctrl+C), pára os outros.
# O Ctrl+C pára tudo da mesma forma (o processamento fecha o bag e grava o resumo); o script continua
# depois (o "trap" impede que o Ctrl+C o termine), para mostrar o resumo.
trap 'true' INT
if [ -n "$BAG_PEDIDO" ]; then
    compose --profile reproduzir up --abort-on-container-exit --no-deps "${servicos[@]}" || true
else
    compose up --abort-on-container-exit --no-deps "${servicos[@]}" || true
fi
trap - INT
if [ "$RVIZ" = "1" ]; then
    compose --profile rviz rm -sf rviz >/dev/null 2>&1 || true
fi

echo
if [ ! -f "$PASTA_SESSAO/resumo_online.json" ]; then
    echo "Não foi gravado o resumo ($PASTA_SESSAO/resumo_online.json): o processamento não chegou a arrancar?"
    exit 1
fi
if [ -n "$BAG_PEDIDO" ]; then
    echo "Resumo da reprodução em: $PASTA_SESSAO/"
elif ls "$PASTA_SESSAO"/*.mcap >/dev/null 2>&1; then
    echo "Gravação (só os trechos) e resumo em: $PASTA_SESSAO/"
else
    echo "Nenhum trecho gravado (só o resumo das deteções) em: $PASTA_SESSAO/"
fi
if [ "$SEM_INTERFACE" = "1" ]; then
    echo "(sem janela: o resumo não é mostrado; está no resumo_online.json)"
else
    echo "A abrir o resumo (feche a janela para terminar)..."
    compose run --rm --no-deps visualizador python3 resumo_online.py "$PASTA_SESSAO" || true
fi
