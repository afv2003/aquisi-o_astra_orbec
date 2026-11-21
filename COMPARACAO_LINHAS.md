# Comparação de modelos de deteção de linhas: CLRNet vs UFLDv2

> Ramo `comparacao-linhas` (fica à parte do `main`). Tudo o resto do projeto (README.md) continua igual; este ramo só acrescenta os ficheiros da comparação.

Ferramenta para comparar, **no mesmo vídeo e na mesma imagem**, o modelo de linhas do projeto (**CLRNet ResNet34**, secção 5.1 do README) com o **UFLDv2 ResNet34** (Ultra-Fast-Lane-Detection-v2), os dois treinados no mesmo conjunto de dados (CULane) e com o mesmo backbone. Segue o processamento offline (`processar_bag.py`, secção 3.9 do README): lê um bag da aquisição (ou do online), uma câmara, corrige a distorção e mostra o vídeo **duas vezes, lado a lado**: à esquerda com as linhas do CLRNet, à direita com as do UFLDv2.

```
 bag .mcap (câmara 5 ou 8)          ┌──▶ CLRNet (linhas.py) ──▶ esquerda ─┐
 ou vídeo / pasta de imagens ──▶ correção ──┤ (a MESMA imagem)                ├──▶ janela lado a lado
                                 (só bags)  └──▶ UFLDv2 (ufld.py)  ──▶ direita ──┘
                                                       │
            registo por trechos (R): deteccoes.mcap, um tópico por modelo ─────┐
            pausa (Espaço): instantes/<n>_<mm>m<ss.ss>s/ (PNG + deteções) ─────┼──▶ resumo_comparacao.py
                                                                               │    (no fim, ou depois)
```

## Índice
1. [Preparar o modelo UFLDv2 (uma vez por PC)](#1-preparar-o-modelo-ufldv2-uma-vez-por-pc)
2. [Comparar](#2-comparar-comparar_linhaspy)
3. [Instantes guardados nas pausas](#3-instantes-guardados-nas-pausas)
4. [Registo das deteções](#4-registo-das-deteções)
5. [Resumo da comparação](#5-resumo-da-comparação-resumo_comparacaopy)
6. [Como funciona o UFLDv2 aqui](#6-como-funciona-o-ufldv2-aqui)
7. [Coerência entre vídeo, deteções e imagens](#7-coerência-entre-vídeo-deteções-e-imagens)
8. [Referência rápida](#8-referência-rápida)

---

## 1. Preparar o modelo UFLDv2 (uma vez por PC)
O ONNX do UFLDv2 ResNet34 CULane tem várias centenas de MB (a última camada é muito grande), por isso **não vai para o git** (o GitHub não aceita ficheiros com mais de 100 MB; o `modelos/ufld.onnx` está no `.gitignore`). Cada PC prepara-o uma vez, com o `venv` do projeto ativo (README, 3.8):
```bash
python preparar_ufld.py
```
* Descarrega o arquivo do **PINTO_model_zoo** (`324_Ultra-Fast-Lane-Detection-v2`, o ONNX indicado no README oficial do UFLDv2), lê-o à medida que chega e guarda só o `ufldv2_culane_res34_320x1600.onnx`, como `modelos/ufld.onnx`.
* Depois **verifica** o modelo: entrada `[1, 3, 320, 1600]` e as quatro saídas do UFLDv2 (`loc_row [1, 200, 72, 4]`, `loc_col [1, 100, 81, 4]`, `exist_row [1, 2, 72, 4]`, `exist_col [1, 2, 81, 4]`), corre-o numa imagem e mostra o tempo por imagem, o dispositivo (CPU/GPU) e o `sha256`. No fim deve aparecer **`MODELO UFLD PRONTO`**.
* Se a descarga falhar (rede, firewall), descarregue o arquivo noutro programa e use-o:
  ```bash
  curl -L -o resources.tar.gz https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/324_Ultra-Fast-Lane-Detection-v2/resources.tar.gz
  python preparar_ufld.py --de resources.tar.gz
  ```
  (`--de` também aceita o `.onnx` já extraído.) Para só verificar: `python preparar_ufld.py --verificar`.
* A configuração do modelo (âncoras, corte, critérios) está em `modelos/ufld.json` (vai para o git).

## 2. Comparar (`comparar_linhas.py`)
Com o `venv` ativo, na pasta do projeto:
```bash
python comparar_linhas.py data/viagem_<data>/viagem_<data>_0.mcap                 # câmara 5 (a primeira do bag)
python comparar_linhas.py data/viagem_<data>/viagem_<data>_0.mcap --camara 8       # câmara 8
python comparar_linhas.py <bag>.mcap --registar-tudo                               # registar o vídeo todo
python comparar_linhas.py <bag>.mcap --sem-interface --trechos "10-20,60-"         # sem janela, trechos fixos
python comparar_linhas.py video.mp4                                                # vídeo externo
python comparar_linhas.py pasta_com_imagens/ --fps 10                              # pasta de imagens
```
* **Bags** (como no `processar_bag.py`): uma câmara de cada vez (por omissão, a primeira do bag: a 5 num bag com as duas; `--camara 8` para a outra). A imagem é corrigida com o `config/camera_astra_<N>.yaml` (ou `--calib`, ou `--sem-correcao`) **antes** dos dois modelos. Funciona com bags da aquisição (`data/viagem_*`) e do online (`data_realtime_inter/realtime_*`), e com bags sem índice ou com blocos estragados (como no offline).
* **Vídeos e pastas de imagens** (como no `processar_video_teste.py`): `.mp4`, `.avi`, `.mov`, `.mkv`, `.m4v`, `.webm`, ou uma pasta com `.jpg`/`.png`/`.bmp`; reduzidos a 640 px de largura (`--largura`, como as Astra; `0` não reduz), sem correção da distorção (a calibração é das Astra). Ritmo: o do vídeo, o tempo no nome dos ficheiros (ex. Argoverse) ou `--fps`.
* **Janela:** o vídeo duas vezes, lado a lado (**CLRNet à esquerda, UFLDv2 à direita**), com as linhas a amarelo (as mesmas cores do projeto) e o painel de cada um: tempo, modelo, câmara e correção, número de linhas e **tempo do modelo (ms)**. Os dois recebem **exatamente a mesma imagem**, por isso as diferenças são só dos modelos.
* **Teclas** (as do offline): **Espaço** pausa (e **guarda o instante**, secção 3) / retoma; **R** começa/termina um trecho do registo; **C** liga/desliga a correção da distorção (bags); **Q** ou **Esc** termina. Sem janela (`--sem-interface`), **Espaço**, **R** e **Q** funcionam no terminal.
* **Parâmetros de cada modelo:** CLRNet com `--conf-linhas` e `--corte-linhas` (os do offline); UFLDv2 com `--corte-ufld` (por omissão 0.4, o corte do treino; secção 6).
* No fim: o tempo médio de cada modelo, a pasta da sessão e, com janela, abre o **resumo** (secção 5; `--sem-resumo` para não abrir).
* As sessões ficam em `data_comparacao/<viagem ou vídeo>/<câmara ou vídeo>_<data_hora>/` (`--pasta-registo` para outra pasta; `data_comparacao/` está no `.gitignore`).

## 3. Instantes guardados nas pausas
Em vez de gravar o vídeo inteiro com as deteções, **cada pausa** (Espaço) guarda o instante em que o vídeo parou (os dois lados param ao mesmo tempo, na mesma imagem), numa pasta só desse instante:
```
data_comparacao/<origem>/<sessão>/instantes/001_01m12.37s/
    original.png      a imagem tal como está no bag (ou no vídeo, já reduzida a 640 px)
    corrigida.png     a imagem corrigida da distorção: a que os DOIS modelos receberam (só nos bags com correção)
    clrnet.png        a vista do CLRNet (linhas e painel), como no ecrã, à esquerda
    ufld.png          a vista do UFLDv2, como no ecrã, à direita
    comparacao.png    o print do ecrã: as duas vistas lado a lado
    deteccoes.json    as deteções dos dois modelos nesse instante e a ligação ao vídeo original
```
* O nome da pasta tem o número do instante e o tempo desde o início do vídeo (`001_01m12.37s`); os instantes da mesma sessão ficam todos na mesma pasta `instantes/`.
* O `deteccoes.json` tem: o instante (`stamp_ns`, segundos, número da imagem, trecho), a fonte (bag e tópico, ou vídeo), a câmara, o tamanho da imagem, qual é a imagem a que as coordenadas se referem (`corrigida.png` ou `original.png`) e as deteções de cada modelo, no mesmo formato do registo (secção 4).
* Um instante é guardado mesmo fora de um trecho do registo (o `deteccoes.json` diz se o instante foi registado).

## 4. Registo das deteções
Igual ao do `processar_bag.py` (README, 3.11 e 3.12): por **trechos** (tecla **R**, `--trechos`, `--registar-tudo`), com as deteções **de cada modelo em separado, como se fossem duas câmaras**:
```
data_comparacao/<origem>/<sessão>/
    deteccoes.mcap    um tópico por modelo: /clrnet/processamento/dados_deteccoes e /ufld/processamento/dados_deteccoes
    ensaio.json       fonte, câmara, calibração, os dois modelos (descrição, dispositivo, parâmetros), trechos e tempos
    instantes/        (secção 3)
```
* Cada mensagem é o JSON do registo offline (`linhas`, `stamp_ns`, `camara`, `corrigida`, `trecho`, `modelos_ativos`), mais `"modelo": "clrnet"` ou `"ufld"`. As duas mensagens de cada imagem têm **o mesmo `stamp_ns`** (o instante da imagem no bag; num vídeo, o tempo desde o início).
* Nos trechos, `imagens_por_camara` conta as imagens de cada modelo (são sempre iguais: os dois processam todas as imagens).
* Leitura: `comparacao.ler_deteccoes_modelos(<deteccoes.mcap>)` devolve `{"clrnet": {stamp_ns: dados}, "ufld": {...}}`.

## 5. Resumo da comparação (`resumo_comparacao.py`)
Abre no fim do `comparar_linhas.py` (com janela) ou depois:
```bash
python resumo_comparacao.py                                   # a sessão mais recente
python resumo_comparacao.py data_comparacao/<origem>/<sessão>
```
Com o aspeto do Resumo do offline e um filtro por **trecho**:
* **Vídeo e registo:** entrada, câmara, correção, trechos, imagens e duração do que está selecionado.
* **Um cartão por modelo** (CLRNet à esquerda, UFLDv2 à direita, como no vídeo): descrição, dispositivo, tempo médio por imagem, parâmetros, linhas por imagem, % de imagens sem linhas e a **distribuição do número de linhas** (0 a 4).
* **Concordância** entre os dois, nas imagens registadas:
  * % de imagens com o **mesmo número de linhas** e a diferença média;
  * % das linhas de cada modelo que **o outro também encontrou** (duas linhas são "a mesma" quando a distância média entre elas, em x, na parte em que as duas existem, é de 30 px ou menos, na imagem que os modelos receberam: `LIMIAR_PX` no `comparacao.py`);
  * a **distância média** entre as linhas correspondentes;
  * % de imagens em que **só um** dos dois encontrou linhas.

  Não é uma medida de qualidade (não há linhas de referência marcadas à mão): mostra onde os modelos concordam e onde diferem; os instantes ajudam a ver quem tem razão.
* **Instantes guardados:** a lista (tempo, linhas de cada modelo, trecho), com um botão para abrir a pasta de cada um.

## 6. Como funciona o UFLDv2 aqui
* **Modelo:** UFLDv2 ResNet34 treinado no CULane (F1 76.0; o CLRNet ResNet34 do projeto tem F1 78.99 no mesmo conjunto). Ficheiros: `ufld.py` (detetor), `modelos/ufld.json` (configuração), `modelos/ufld.onnx` (preparado na secção 1).
* **Ideia do UFLD:** em vez de procurar a linha em cada píxel, escolhe, para cada uma de 72 linhas horizontais da imagem (âncoras), em que coluna (de 200) passa cada uma de 4 linhas da estrada; as duas linhas de fora usam 81 âncoras verticais. Por isso é muito rápido nas GPUs, mas mais grosseiro do que o CLRNet.
* **Pré-processamento** (igual ao demo oficial, `demo.py` e `configs/culane_res34.py`): RGB; corta-se o topo da imagem (40%: o modelo foi treinado a ver só os 60% de baixo, `crop_ratio = 0.6`); redimensiona-se para 1600 x 320; normaliza-se com a média e o desvio do ImageNet.
* **Pós-processamento** (o `pred2coords` oficial, em numpy): uma linha do meio conta se existir em **mais de metade** das 72 âncoras; uma de fora, em **mais de um quarto** das 81; a posição em cada âncora é a média pesada (softmax) da célula mais provável e das vizinhas. As linhas saem da esquerda para a direita, em píxeis da imagem recebida.
* **`--corte-ufld`:** com outro corte do topo, as posições do modelo (frações da imagem "do treino") são convertidas para a imagem real; o valor do treino (0.4) é o mais seguro.
* **Diferenças em relação ao CLRNet** a ter em conta na comparação: o UFLDv2 não dá uma confiança por linha (só "existe/não existe" em cada âncora), por isso não há limiar de confiança equivalente ao `--conf-linhas`; o corte do topo por omissão também é diferente (0.4 no UFLDv2, 0.46 no CLRNet), porque cada modelo usa o do seu treino.
* **Imagens das Astra (640x480, 4:3)** são mais "altas" do que as do CULane (1640x590): os dois modelos esticam a imagem na horizontal, da mesma forma que no projeto (README, 5.1.5); as condições de utilização do CLRNet (câmara virada para a frente, horizonte a meio) aplicam-se também ao UFLDv2.

## 7. Coerência entre vídeo, deteções e imagens
O que garante que o vídeo original, as deteções e os PNG são do mesmo instante e da mesma imagem:
* **Uma imagem, dois modelos:** em cada instante, a imagem é lida e corrigida **uma vez** e a mesma imagem é passada aos dois modelos; os desenhos são feitos em cópias.
* **O mesmo instante em todo o lado:** o `stamp_ns` (o instante da imagem no bag) é o mesmo nas duas mensagens do registo, no `deteccoes.json` do instante e no nome da pasta (em segundos desde o início).
* **PNG = imagem do bag:** o `original.png` é a imagem do bag com esse `stamp_ns`, sem alterações; o `corrigida.png` é essa imagem corrigida com a calibração guardada no `ensaio.json`; as coordenadas das deteções referem-se à `imagem_dos_modelos` indicada no `deteccoes.json`.
* **Deteções do instante = deteções do registo:** num instante dentro de um trecho, as deteções do `deteccoes.json` são as mesmas que ficam no `deteccoes.mcap` com esse `stamp_ns`.
* **`comparacao.png` = `clrnet.png` | `ufld.png`:** o print lado a lado é feito com as duas vistas guardadas.

Isto foi verificado nos testes: para cada instante, o `original.png` é igual, píxel a píxel, à imagem do bag com o mesmo `stamp_ns`, o `corrigida.png` é igual à sua correção, as deteções do `deteccoes.json` são iguais às do registo e o `comparacao.png` é igual às duas vistas juntas.

## 8. Referência rápida

| Comando | Opção | O que faz (valor por omissão) |
|---|---|---|
| `python preparar_ufld.py` | (nenhuma) | descarrega, extrai e verifica o `modelos/ufld.onnx` |
| | `--de <resources.tar.gz ou .onnx>` | usa um ficheiro já descarregado |
| | `--verificar` | só verifica o `modelos/ufld.onnx` |
| `python comparar_linhas.py <entrada>` | `<entrada>` | bag `.mcap`, vídeo ou pasta de imagens |
| | `--camara 5` / `--camara 8` | câmara do bag (a primeira) |
| | `--calib <yaml>`, `--sem-correcao` | outra calibração / sem correção (bags) |
| | `--conf-linhas`, `--corte-linhas` | CLRNet: confiança mínima (0.4) e corte do topo (0.46) |
| | `--corte-ufld` | UFLDv2: corte do topo (0.4) |
| | `--fps`, `--largura` | vídeos e pastas: ritmo (o do vídeo / 30) e largura (640) |
| | `--sem-interface` | sem janela; teclas no terminal |
| | `--trechos "10-20,60-"`, `--registar-tudo` | trechos do registo |
| | `--pasta-registo <pasta>` | pasta das sessões (`data_comparacao`) |
| | `--sem-resumo` | não abrir o resumo no fim |
| `python resumo_comparacao.py` | `[<sessão>]`, `--pasta-registo` | resumo de uma sessão (a mais recente) |

| Tecla (janela do `comparar_linhas.py`) | O que faz |
|---|---|
| **Espaço** | pausa e guarda o instante / retoma |
| **R** | começa / termina um trecho do registo |
| **C** | liga / desliga a correção da distorção (bags) |
| **Q** / **Esc** | termina |

**Ficheiros deste ramo:** `comparar_linhas.py`, `ufld.py`, `comparacao.py`, `resumo_comparacao.py`, `preparar_ufld.py`, `modelos/ufld.json` e este documento. Usam, sem alterações, os módulos do projeto: `linhas.py`, `leitura_bag.py`, `registo.py`, `visual.py`, `gui_comum.py` e `teclado.py`.

**Créditos:** [Ultra-Fast-Lane-Detection-v2](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2), Qin et al., *Ultra Fast Deep Lane Detection with Hybrid Anchor Driven Ordinal Classification*, TPAMI 2022 (licença MIT); conversão para ONNX: [PINTO_model_zoo](https://github.com/PINTO0309/PINTO_model_zoo/tree/main/324_Ultra-Fast-Lane-Detection-v2) (PINTO0309).
