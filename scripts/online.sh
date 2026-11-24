#!/usr/bin/env bash
# Online (Docker): ao vivo, com as câmaras, ou em reprodução de um bag, sem câmaras.
#
#   bash scripts/online.sh 5 8                  ao vivo: câmara 5 em /dev/video2, câmara 8 em /dev/video4
#   bash scripts/online.sh --bag <bag.mcap>     reprodução: as imagens vêm do bag (mesmo processamento)
#
# Serviços (compose.yaml): drivers das câmaras (perfil sensor) ou reprodutor (perfil reproduzir),
# percecao, planeador, visualizador (a janela, perfil visualizador) e, a pedido, RViz, Foxglove e gravador.
#
# Antes (README, secção 4.6): a imagem construída (docker compose build, com o ONNX_PKG do .env). Este
# script não a constrói. Pode correr-se de qualquer pasta: os caminhos (incluindo o do --bag) são
# relativos à pasta do terminal; o compose.yaml, o .env e a pasta dos dados são os do repositório.
set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAMARAS_VALIDAS="5 8"
read -r -a DEVS <<< "${DEVS_VIDEO:-/dev/video2 /dev/video4}"

uso() {
    echo "Uso:"
    echo "  Ao vivo:     bash scripts/online.sh <câmara no ${DEVS[0]}> [<câmara no ${DEVS[1]}>] [opções]"
    echo "  Reprodução:  bash scripts/online.sh --bag <ficheiro .mcap> [opções]"
    echo "Opções:"
    echo "  -i                  : sem janela (sem o visualizador; corre só no terminal)"
    echo "  --rviz              : abre também o RViz (TF das câmaras, grelha local, caminho do A*, imagens)"
    echo "  --foxglove          : lança também o foxglove_bridge (Foxglove Studio em ws://localhost:8765)"
    echo "  --gravar            : grava também os resultados (ros2 bag: grelha, avisos, caminho) em data/"
    echo "  --sem-correcao      : não corrige a distorção da lente (por omissão usa config/camera_astra_<N>.yaml)"
    echo "  ao vivo:"
    echo "  --percurso <nome>   : junta o nome do percurso ao da sessão (g05_camaras_<data_hora>_<nome>)"
    echo "  --registar-tudo     : começa logo um trecho (grava a sessão toda; a tecla R continua a funcionar)"
    echo "  --sem-processamento : só as câmaras e a gravação dos trechos (sem os modelos; para PCs mais fracos)"
    echo "  em reprodução:"
    echo "  --taxa <x>          : velocidade (1 = tempo real, 0.5 = metade, 2 = o dobro)"
    echo "  --camara <N>        : reproduzir só uma das câmaras do bag"
    echo "Antes: a imagem construída (docker compose build; README, secção 4.6)."
    echo "Ao vivo, a deteção corre sempre; o vídeo original (JPEG) e as deteções só são gravados nos trechos:"
    echo "  tecla R na janela, ou, sem janela, bash scripts/trecho.sh (noutro terminal)."
    echo "  ex.: bash scripts/online.sh 5 8"
    echo "  ex.: bash scripts/online.sh 5 8 -i --registar-tudo --percurso ua_ria"
    echo "  ex.: bash scripts/online.sh --bag data/g05_camaras_20261124_101500/g05_camaras_20261124_101500_0.mcap"
    echo "  Para forçar CPU num PC com GPU: USAR_GPU=0 bash scripts/online.sh 5 8"
    echo "  Sem janela, para desligar/ligar a câmara da direita: bash scripts/alternar_camara.sh (noutro terminal)"
    exit 1
}

# valor de uma variável: do ambiente, senão do .env do repositório, senão o valor por omissão
valor_env() {
    local nome="$1" omissao="$2" v
    v="${!nome:-}"
    if [ -z "$v" ] && [ -f "$RAIZ/.env" ]; then
        v=$(sed -n "s/^${nome}=//p" "$RAIZ/.env" | tail -n1 | tr -d '\r')
    fi
    echo "${v:-$omissao}"
}

export SEM_INTERFACE=0 SEM_CORRECAO=0 REGISTAR_TUDO=0 SEM_PROCESSAMENTO=0 REPRODUCAO="" TAXA=1
RVIZ=0
FOXGLOVE=0
GRAVAR=0
BAG_PEDIDO=""
CAMARA_BAG=""
PERCURSO=""
camaras=()

while [ $# -gt 0 ]; do
    case "$1" in
        -i|--sem-interface) SEM_INTERFACE=1 ;;
        --sem-correcao) SEM_CORRECAO=1 ;;
        --registar-tudo) REGISTAR_TUDO=1 ;;
        --sem-processamento) SEM_PROCESSAMENTO=1 ;;
        --rviz) RVIZ=1 ;;
        --foxglove) FOXGLOVE=1 ;;
        --gravar) GRAVAR=1 ;;
        --bag) [ $# -ge 2 ] || uso; BAG_PEDIDO="$2"; shift ;;
        --taxa) [ $# -ge 2 ] || uso; TAXA="$2"; shift ;;
        --camara) [ $# -ge 2 ] || uso; CAMARA_BAG="$2"; shift ;;
        --percurso) [ $# -ge 2 ] || uso; PERCURSO="$2"; shift ;;
        -h|--help) uso ;;
        -*) echo "Erro: opção desconhecida '$1'."; uso ;;
        *) camaras+=("$1") ;;
    esac
    shift
done

# --- Pasta dos dados (DADOS no .env, relativa ao repositório): a mesma que o compose monta em /data ---
DADOS="$(valor_env DADOS ./data)"
case "$DADOS" in /*) ;; *) DADOS="$RAIZ/${DADOS#./}" ;; esac
mkdir -p "$DADOS"
export DADOS

# --- Docker: CPU ou GPU NVIDIA (omissão: a GPU, se existir; forçar CPU com USAR_GPU=0) ---
COMPOSE_FILES=(-f "$RAIZ/compose.yaml")
ONNX_PKG="$(valor_env ONNX_PKG onnxruntime)"
if [ "$SEM_PROCESSAMENTO" = "1" ]; then
    echo "Hardware: sem processamento (os modelos não são usados)"
elif [ "${USAR_GPU:-auto}" != "0" ] \
    && command -v nvidia-smi >/dev/null 2>&1 \
    && nvidia-smi >/dev/null 2>&1 \
    && docker info 2>/dev/null | grep -qi nvidia; then
    COMPOSE_FILES+=(-f "$RAIZ/compose.gpu.yaml")
    CUDA_MAJOR=$(nvidia-smi 2>/dev/null | sed -n 's/.*CUDA Version: \([0-9]\+\).*/\1/p' | head -n1 || true)
    echo "Hardware detetado: GPU NVIDIA (CUDA ${CUDA_MAJOR:-?}); imagem com ${ONNX_PKG}"
    case "$ONNX_PKG" in
        onnxruntime-gpu*) ;;
        *) echo "  Aviso: a imagem tem o onnxruntime de CPU: os modelos correm no CPU. Para a GPU, mude o"
           echo "  ONNX_PKG no .env e construa de novo a imagem (docker compose build; README, secção 4.3)." ;;
    esac
else
    if [ "${USAR_GPU:-auto}" = "0" ]; then
        echo "Hardware: CPU (forçado por USAR_GPU=0)"
    elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
        echo "Hardware: GPU NVIDIA encontrada, mas falta o NVIDIA Container Toolkit no Docker -> a usar CPU"
    else
        echo "Hardware detetado: CPU"
    fi
fi
# o compose precisa destas variáveis em todos os modos (os serviços que não arrancam não as usam)
export DEV_CAMARA5=/dev/null DEV_CAMARA8=/dev/null BAG="" PASTA_BAG="$DADOS" RELOGIO_NTP="" CAMARAS_ATIVAS=""
compose() { docker compose "${COMPOSE_FILES[@]}" "$@"; }

servicos=()
perfis=(--profile visualizador)
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
    perfis+=(--profile reproduzir)
    # câmaras e trechos do bag (lidos dentro do Docker, como o reprodutor os vai ler)
    INFO=$(compose --profile reproduzir run --rm --no-deps -T reprodutor reprodutor "/bag/$BAG" --info) || {
        echo "Erro: o bag não tem imagens de câmaras que se possam reproduzir."; exit 1; }
    CAMARAS_ATIVAS=$(printf '%s\n' "$INFO" | sed -n 's/^CAMARAS=//p')
    [ -n "$CAMARAS_ATIVAS" ] || { echo "Erro: nenhuma câmara no bag${CAMARA_BAG:+ (pedida: $CAMARA_BAG)}."; exit 1; }
    export CAMARAS_ATIVAS
    export REPRODUCAO="$BAG"
    export SESSAO="g05_reproducao_$(basename "$BAG" .mcap)_$(date +%Y%m%d_%H%M%S)"
    servicos=(reprodutor percecao planeador)
    echo "Modo: REPRODUÇÃO de $BAG_PEDIDO (câmara(s) $CAMARAS_ATIVAS; velocidade x$TAXA)"
    echo "  a perceção, o planeador e o visualizador são os mesmos do ao vivo; nada é gravado"
else
    # ------------------------------------------------------------ AO VIVO (câmaras)
    [ ${#camaras[@]} -ge 1 ] && [ ${#camaras[@]} -le 2 ] || uso
    if [ ${#camaras[@]} -eq 2 ] && [ "${camaras[0]}" = "${camaras[1]}" ]; then
        echo "Erro: indicou a câmara ${camaras[0]} duas vezes."
        exit 1
    fi
    export CAMARAS_ATIVAS="${camaras[*]}"
    perfis+=(--profile sensor)
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
    servicos+=(percecao)
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
    PERCURSO=$(printf '%s' "$PERCURSO" | tr -c 'A-Za-z0-9_-' '_')
    export SESSAO="g05_camaras_$(date +%Y%m%d_%H%M%S)${PERCURSO:+_$PERCURSO}"
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
    echo "Ecrã: SEM janela (-i)  -  trechos: bash scripts/trecho.sh, noutro terminal"
else
    servicos+=(visualizador)
    echo "Ecrã: janela do visualizador (R trecho, D câmara da direita, Espaço pausa na reprodução, Q terminar)"
fi
PASTA_SESSAO="$DADOS/$SESSAO"
if [ -n "$BAG_PEDIDO" ]; then
    echo "Resumo da reprodução em: $PASTA_SESSAO/"
else
    echo "Gravação (vídeo original em JPEG + deteções, só nos trechos) e resumo em: $PASTA_SESSAO/"
fi
echo

# O RViz, o foxglove_bridge e o gravador arrancam à parte: fechá-los não termina a sessão
extras=()
if [ "$RVIZ" = "1" ]; then
    if compose --profile rviz up -d --no-deps rviz; then extras+=(rviz); else echo "Aviso: o RViz não arrancou."; fi
fi
if [ "$FOXGLOVE" = "1" ]; then
    if compose --profile foxglove up -d --no-deps foxglove; then
        extras+=(foxglove)
        echo "Foxglove: no Foxglove Studio, abrir ligação Foxglove WebSocket em ws://localhost:8765"
        echo "  (layout dos painéis: docs/layout.json)"
    else
        echo "Aviso: o foxglove_bridge não arrancou."
    fi
fi
if [ "$GRAVAR" = "1" ]; then
    if compose --profile gravar up -d --no-deps gravador; then
        extras+=(gravador)
        echo "Resultados (ros2 bag) em: $DADOS/g05_resultados_<data_hora>/"
    else
        echo "Aviso: o gravador não arrancou."
    fi
fi

# --abort-on-container-exit: ao terminar a perceção (tecla Q, fim do bag, Ctrl+C), pára os outros.
# O Ctrl+C pára tudo da mesma forma (a perceção fecha o bag e grava o resumo); o script continua
# depois (o "trap" impede que o Ctrl+C o termine), para mostrar o resumo.
trap 'true' INT
compose "${perfis[@]}" up --abort-on-container-exit --no-deps "${servicos[@]}" || true
trap - INT
if [ ${#extras[@]} -gt 0 ]; then
    # o gravador recebe o SIGINT do "stop" e fecha o bag antes de ser removido
    compose --profile rviz --profile foxglove --profile gravar stop "${extras[@]}" >/dev/null 2>&1 || true
    compose --profile rviz --profile foxglove --profile gravar rm -f "${extras[@]}" >/dev/null 2>&1 || true
fi

echo
if [ ! -f "$PASTA_SESSAO/resumo_online.json" ]; then
    echo "Não foi gravado o resumo ($PASTA_SESSAO/resumo_online.json): a perceção não chegou a arrancar?"
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
    compose --profile visualizador run --rm --no-deps visualizador resumo_online "/data/$SESSAO" || true
fi
