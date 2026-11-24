# Comparação de modelos de deteção de linhas: CLRNet vs UFLDv2

> Ramo `Extra-Line-Comparison`: fica **à parte do `main`** e não é para juntar ao `main`. Tudo o resto do projeto (README.md) é igual ao do `main`; este ramo só acrescenta a pasta `comparacao_linhas/` e este documento (secção 11). Corre no `venv` do offline, sem Docker.

Ferramenta para comparar, **no mesmo vídeo e na mesma imagem**, o modelo de linhas do projeto (**CLRNet ResNet34**, README 5.1) com o **UFLDv2 ResNet34** (Ultra-Fast-Lane-Detection-v2). Os dois foram treinados no mesmo conjunto de dados (CULane) e têm o mesmo backbone. Segue o processamento offline (`offline/processar_bag.py`, README 3.4): lê o vídeo de uma câmara, corrige a distorção e mostra-o **duas vezes, lado a lado**: à esquerda com as linhas do CLRNet, à direita com as do UFLDv2. É uma ferramenta **demonstrativa**: em vez de gravar o vídeo inteiro com as deteções, guarda em `.png` os instantes em que se pausa o vídeo.

```
 bag .mcap (câmara 5 ou 8)              ┌──▶ CLRNet (linhas.py) ──▶ esquerda ─┐
 ou vídeo / pasta de imagens ──▶ correção ──┤     (a MESMA imagem)                ├──▶ janela lado a lado
                                 (só bags)  └──▶ UFLDv2 (ufld.py)  ──▶ direita ──┘
                                                          │
            registo por trechos (R): deteccoes.mcap, um tópico por modelo ─────┐
            pausa (Espaço): instantes/<n>_<mm>m<ss.ss>s/ (PNG + deteções) ─────┼──▶ resumo_comparacao.py
                                                                               │    (no fim, ou depois)
```

## Índice
0. [Onde encaixa no projeto e quando usar](#0-onde-encaixa-no-projeto-e-quando-usar)
1. [Preparar o ambiente e o modelo UFLDv2 (uma vez por PC)](#1-preparar-o-ambiente-e-o-modelo-ufldv2-uma-vez-por-pc)
2. [Escolher a entrada: bag, vídeo ou pasta de imagens](#2-escolher-a-entrada-bag-vídeo-ou-pasta-de-imagens)
3. [Comparar (`comparar_linhas.py`)](#3-comparar-comparar_linhaspy)
4. [Instantes guardados nas pausas](#4-instantes-guardados-nas-pausas)
5. [Registo das deteções (trechos)](#5-registo-das-deteções-trechos)
6. [Resumo da comparação (`resumo_comparacao.py`)](#6-resumo-da-comparação-resumo_comparacaopy)
7. [Treino dos modelos, métricas e perspetiva do vídeo](#7-treino-dos-modelos-métricas-e-perspetiva-do-vídeo)
8. [Desempenho e GPU](#8-desempenho-e-gpu)
9. [Coerência entre vídeo, deteções e imagens](#9-coerência-entre-vídeo-deteções-e-imagens)
10. [Referência rápida (opções e teclas)](#10-referência-rápida-opções-e-teclas)
11. [Ficheiros e manutenção do ramo](#11-ficheiros-e-manutenção-do-ramo)

---

## 0. Onde encaixa no projeto e quando usar
**Ponto comum com o resto do projeto: o vídeo original em `.mcap`.** Tal como o modo offline e o modo online (README 1.1), a comparação parte do vídeo original das câmaras gravado num bag:
```
 online (aquisição, README 4) ──▶ data/g05_camaras_<data_hora>/*.mcap (vídeo, só os trechos) ───┐
 bags antigos (README 3.1) ─────▶ data/viagem_<data>/*.mcap (vídeo da viagem inteira) ──────────┼──▶ comparacao_linhas/comparar_linhas.py
 vídeos / imagens externos (ex. nuScenes, Argoverse) ───────────────────────────────────────────┘
```
* Lê os bags **da mesma forma que o `processar_bag.py`** (`leitura_bag.py`): mesma câmara, mesma calibração, mesma correção da distorção e mesmo `stamp_ns` de cada imagem. Por isso, o que se vê aqui é o que o CLRNet do offline e do online vê.
* As deteções ficam no **mesmo formato do registo do offline** (README 3.6-3.7), com cada modelo no lugar de uma câmara (secção 5).
* Vídeos e pastas de imagens externos são lidos **como no `processar_video_teste.py`** (o script de testes do offline com vídeos externos, por exemplo do Argoverse 2 ou do nuScenes).

**Diferenças em relação ao `processar_bag.py`:**

| | `processar_bag.py` (offline) | `comparar_linhas.py` (este ramo) |
|---|---|---|
| Modelos | objetos, linhas (CLRNet) e estrada | só linhas: CLRNet **e** UFLDv2, na mesma imagem |
| Câmaras | 5 e 8 ao mesmo tempo | uma de cada vez (`--camara`) |
| Vídeo com deteções | grava o vídeo inteiro | **não grava**: guarda `.png` dos instantes de pausa |
| Registo | um tópico por câmara | um tópico por modelo (como duas câmaras) |
| Janela no fim | Resumo e Percurso (`gui_resultados.py`) | resumo simplificado da comparação |

**Quando usar:**
* para **escolher ou justificar o modelo de linhas**, olhando para as mesmas imagens com os dois modelos;
* para **demonstrar** casos concretos (curvas, linhas apagadas, sombras, noite): pausa-se e fica o `.png` lado a lado;
* para testar os dois modelos em **conjuntos de dados públicos** antes de os testar na estrada.

Para processar uma viagem com todas as deteções, use o `processar_bag.py` (offline); para processar em tempo real no veículo, o modo online (README 4; quando usar cada modo: README 1.2).

**Relevância para a parte 2 (online, README 1.3):** o online usa o CLRNet e precisa de imagens por segundo. O UFLD é conhecido por ser rápido; esta comparação mostra, nas mesmas imagens e na mesma GPU, quanto custa cada modelo em tempo (secção 8) e onde diferem as deteções (secções 4, 6 e 7.2), antes de pensar em mudar o modelo do online.

## 1. Preparar o ambiente e o modelo UFLDv2 (uma vez por PC)
1. **Ambiente Python:** o mesmo `venv` do offline (README 3.3; GPU: README 3.5). Ative-o **em cada terminal novo**:
   ```bash
   venv\Scripts\activate          # Windows
   source venv/bin/activate       # Linux / macOS
   ```
   Sem o `venv` ativo aparece `falta o onnxruntime` (o `preparar_ufld.py` avisa logo, antes de descarregar).
2. **Ramo:**
   ```bash
   git switch Extra-Line-Comparison
   ```
3. **Modelo UFLDv2.** O ONNX do UFLDv2 ResNet34 CULane tem **866 MB** (a última camada é muito grande), por isso **não vai para o git** (o GitHub não aceita ficheiros com mais de 100 MB; `comparacao_linhas/modelos/ufld.onnx` está no `.gitignore`). Prepara-se uma vez:
   ```bash
   python comparacao_linhas/preparar_ufld.py
   ```
   * Descarrega o arquivo do **PINTO_model_zoo** (`324_Ultra-Fast-Lane-Detection-v2`, o ONNX indicado no README oficial do UFLDv2; cerca de 2,9 GB), lê-o à medida que chega e guarda só o `ufldv2_culane_res34_320x1600.onnx`, como `comparacao_linhas/modelos/ufld.onnx`. Pára de descarregar logo que o encontra (cerca de 1,2 GB lidos).
   * Depois **verifica** o modelo: entrada `[1, 3, 320, 1600]` e as quatro saídas do UFLDv2 (`loc_row [1, 200, 72, 4]`, `loc_col [1, 100, 81, 4]`, `exist_row [1, 2, 72, 4]`, `exist_col [1, 2, 81, 4]`), corre-o numa imagem e mostra o dispositivo (CPU/GPU), o tempo por imagem e o `sha256`. No fim deve aparecer **`MODELO UFLD PRONTO`**.
   * Se a descarga falhar (rede, firewall), descarregue o arquivo noutro programa e use-o:
     ```bash
     curl -L -o resources.tar.gz https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/324_Ultra-Fast-Lane-Detection-v2/resources.tar.gz
     python comparacao_linhas/preparar_ufld.py --de resources.tar.gz
     ```
     (`--de` também aceita o `.onnx` já extraído.)
   * Se a verificação falhar a meio (por exemplo, sem o `venv` ativo), o modelo já descarregado fica guardado: basta `python comparacao_linhas/preparar_ufld.py --verificar`. Se o ficheiro ficou incompleto (erro "não é um modelo ONNX válido"), apague o `comparacao_linhas/modelos/ufld.onnx` e corra de novo.
   * A configuração do modelo (âncoras, corte, critérios) está em `comparacao_linhas/modelos/ufld.json` (esse vai para o git).

## 2. Escolher a entrada: bag, vídeo ou pasta de imagens
O primeiro argumento do `comparar_linhas.py` é a entrada; o tipo é reconhecido sozinho.

**Bags `.mcap`** (como no `processar_bag.py`):
* Da aquisição (`data/viagem_<data>/viagem_<data>_0.mcap`) ou do online (`data_realtime_inter/realtime_<data>/...`). Funciona também com bags sem índice ou com blocos estragados (lê sequencialmente, como no offline).
* **Uma câmara de cada vez**: por omissão a primeira do bag (a 5 num bag com as duas); `--camara 8` para a outra.
* A imagem é **corrigida da distorção antes dos dois modelos** com `src/va_g05/config/camera_astra_<N>.yaml` (ou o `/camera_info` do bag); `--calib <yaml>` para outra calibração, `--sem-correcao` para não corrigir.

**Vídeos** (`.mp4`, `.avi`, `.mov`, `.mkv`, `.m4v`, `.webm`) e **pastas de imagens** (`.jpg`, `.jpeg`, `.png`, `.bmp`), como no `processar_video_teste.py`:
* Reduzidos a **640 px de largura** (`--largura`, como as Astra; `--largura 0` não reduz). Sem correção da distorção (a calibração é a das Astra).
* **Pastas:** as imagens são lidas por ordem do nome. O tempo de cada imagem vem:
  * do nome do ficheiro, se o nome for só o instante em nanossegundos (ex. Argoverse 2: `315967376899927209.jpg`);
  * senão, de `--fps` (por omissão 30 imagens/s). **Indique sempre o `--fps` certo**: os tempos no ecrã, nos trechos e nos nomes das pastas dos instantes vêm daí.
* **nuScenes:** use uma pasta da câmara da frente e **uma cena de cada vez** (a pasta do nuScenes mistura todas as cenas; os ficheiros da mesma cena começam pelo mesmo prefixo, ex. `n008-2018-08-01-15-16-36-0400__CAM_FRONT__*.jpg`; copie-os para uma pasta à parte):
  ```bash
  python comparacao_linhas/comparar_linhas.py data/nuscenes/samples/CAM_FRONT --fps 2      # só keyframes (2 imagens/s)
  python comparacao_linhas/comparar_linhas.py data/nuscenes/sweeps/CAM_FRONT --fps 12      # todas as imagens (cerca de 12/s)
  ```
  A `CAM_FRONT` (1600x900) fica com 640x360. A câmara de trás (`CAM_BACK`) também funciona, mas a vista da frente é a que corresponde às Astra.

## 3. Comparar (`comparar_linhas.py`)
Com o `venv` ativo, na pasta do projeto:
```bash
python comparacao_linhas/comparar_linhas.py data/viagem_<data>/viagem_<data>_0.mcap                 # câmara 5 (a primeira do bag)
python comparacao_linhas/comparar_linhas.py data/viagem_<data>/viagem_<data>_0.mcap --camara 8       # câmara 8
python comparacao_linhas/comparar_linhas.py <bag>.mcap --registar-tudo                               # registar o vídeo todo
python comparacao_linhas/comparar_linhas.py <bag>.mcap --sem-interface --trechos "10-20,60-"         # sem janela, trechos fixos
python comparacao_linhas/comparar_linhas.py video.mp4                                                # vídeo externo
python comparacao_linhas/comparar_linhas.py pasta_com_imagens --fps 10                               # pasta de imagens
```

**3.1 No arranque, o terminal mostra** a configuração de cada modelo, a entrada, a pasta da sessão e as teclas:
```
CLRNet: CLRNet ResNet34, CULane (F1 78.99 no UnLanedet)  ·  GPU (CUDA)  ·  corte do topo 46%  ·  confiança mínima 0.4
UFLDv2: UFLDv2 ResNet34, CULane (F1 76.0)  ·  GPU (CUDA)  ·  corte do topo 40%
Entrada: CAM_FRONT (pasta)  ·  sem correção da distorção  ·  reduzida a 640 px de largura
Sessão: data\comparacao\CAM_FRONT\CAM_FRONT_20261121_033909
```
Confirme o dispositivo (`GPU (CUDA)` ou `CPU`; secção 8).

**3.2 Janela:** o vídeo duas vezes, lado a lado (**CLRNet à esquerda, UFLDv2 à direita**), com as linhas nas cores do projeto e o painel de cada um: tempo, modelo, câmara e correção, número de linhas e **tempo do modelo (ms)**. Com um trecho a registar, aparece `REGISTO · trecho N`. Os dois modelos recebem **exatamente a mesma imagem**, por isso as diferenças são só dos modelos. O vídeo anda ao ritmo da gravação; se os modelos forem mais lentos, anda mais devagar, sem saltar imagens.

**3.3 Teclas** (as do offline):

| Tecla | O que faz |
|---|---|
| **Espaço** | pausa os dois lados ao mesmo tempo e **guarda o instante** (secção 4) / retoma |
| **R** | começa / termina um trecho do registo (secção 5); funciona também durante a pausa |
| **C** | liga / desliga a correção da distorção (só bags com calibração): os dois modelos passam a receber a imagem original, e os instantes e o registo indicam qual receberam |
| **Q** / **Esc** | termina (um trecho aberto é fechado e guardado) |

* Sem janela (`--sem-interface`): **Espaço**, **R** e **Q** no terminal, que mostra o tempo, os ms de cada modelo e o estado do registo.
* Com `--trechos` ou `--registar-tudo`, o registo é automático e a tecla **R** fica desligada.

**3.4 No fim:** o terminal mostra o tempo médio de cada modelo, os trechos registados, os instantes guardados e o comando do resumo (exemplo de uma sessão em CPU):
```
Tempo médio por imagem: CLRNet 184 ms | UFLDv2 296 ms  (117 imagens)
Registo guardado em: data\comparacao\CAM_BACK\CAM_BACK_20261121_033909
  trecho 1: 32.5 s a 49.0 s (34 imagens, os dois modelos)
Instantes guardados: 2, em data\comparacao\CAM_BACK\CAM_BACK_20261121_033909\instantes
Resumo: python comparacao_linhas/resumo_comparacao.py "data\comparacao\CAM_BACK\CAM_BACK_20261121_033909"
```
Com janela, no fim do vídeo aparece "Fim do vídeo" (**Q** fecha) e abre o **resumo** (secção 6; `--sem-resumo` para não abrir), se houver trechos ou instantes.

As sessões ficam em `data/comparacao/<viagem ou entrada>/<câmara ou entrada>_<data_hora>/` (`--pasta-registo` para outra pasta; `data/comparacao/` está no `.gitignore`).

## 4. Instantes guardados nas pausas
Em vez de gravar o vídeo inteiro com as deteções, **cada pausa** (Espaço) guarda o instante em que o vídeo parou (os dois lados param ao mesmo tempo, na mesma imagem), numa pasta só desse instante:
```
data/comparacao/<origem>/<sessão>/instantes/001_01m12.37s/
    original.png      a imagem tal como está no bag (ou no vídeo, já reduzida a 640 px)
    corrigida.png     a imagem corrigida da distorção: a que os DOIS modelos receberam (só bags com correção)
    clrnet.png        a vista do CLRNet (linhas e painel), como no ecrã, à esquerda
    ufld.png          a vista do UFLDv2, como no ecrã, à direita
    comparacao.png    o print do ecrã: as duas vistas lado a lado
    deteccoes.json    as deteções dos dois modelos nesse instante e a ligação ao vídeo original
```
* O nome da pasta tem o número do instante e o tempo desde o início do vídeo (`001_01m12.37s`); os instantes da mesma sessão ficam todos na mesma pasta `instantes/`, por ordem.
* O `deteccoes.json` tem: o instante (`stamp_ns`, segundos, número da imagem, trecho, se foi registado), a fonte (bag e tópico, ou vídeo/pasta e nome do ficheiro), a câmara, o tamanho da imagem, qual é a imagem a que as coordenadas se referem (`imagem_dos_modelos`: `corrigida.png` ou `original.png`) e as deteções de cada modelo, no mesmo formato do registo (secção 5).
* Um instante é guardado mesmo fora de um trecho do registo.

## 5. Registo das deteções (trechos)
Igual ao do `processar_bag.py` (README 3.6 e 3.7): por **trechos** (tecla **R**, `--trechos`, `--registar-tudo`), com as deteções **de cada modelo em separado, como se fossem duas câmaras**:
```
data/comparacao/<origem>/<sessão>/
    deteccoes.mcap    um tópico por modelo: /clrnet/processamento/dados_deteccoes e /ufld/processamento/dados_deteccoes
    ensaio.json       fonte, câmara, calibração, os dois modelos (descrição, dispositivo, parâmetros), trechos e tempos
    instantes/        (secção 4)
```
* Cada mensagem é o JSON do registo offline (`linhas`, `stamp_ns`, `camara`, `corrigida`, `trecho`, `modelos_ativos`), mais `"modelo": "clrnet"` ou `"ufld"`. As duas mensagens de cada imagem têm **o mesmo `stamp_ns`** (o instante da imagem no bag; num vídeo ou pasta, o tempo desde o início).
* `--trechos "10-20,60-"`: segundos desde o início do vídeo (`60-` = dos 60 s até ao fim); `--registar-tudo` = `--trechos 0-`.
* **Pausar a meio de um trecho não estraga o registo:** durante a pausa não se escreve nada; a imagem da pausa fica registada uma só vez e, ao retomar, o mesmo trecho continua na imagem seguinte (sem imagens repetidas nem em falta).
* Nos trechos, `imagens_por_camara` conta as imagens de cada modelo (são sempre iguais: os dois processam todas as imagens).
* Leitura em Python: `comparacao.ler_deteccoes_modelos(<deteccoes.mcap>)` devolve `{"clrnet": {stamp_ns: dados}, "ufld": {...}}`.

## 6. Resumo da comparação (`resumo_comparacao.py`)
Abre no fim do `comparar_linhas.py` (com janela) ou depois:
```bash
python comparacao_linhas/resumo_comparacao.py                                   # a sessão mais recente (em data/comparacao)
python comparacao_linhas/resumo_comparacao.py data/comparacao/<origem>/<sessão>
python comparacao_linhas/resumo_comparacao.py --pasta-registo outra_pasta       # a mais recente noutra pasta
```
Com o aspeto do Resumo do offline (mais simples) e um filtro por **trecho**:
* **Vídeo e registo:** entrada, câmara, correção, trechos, imagens e duração do que está selecionado.
* **Um cartão por modelo** (CLRNet à esquerda, UFLDv2 à direita, como no vídeo): descrição, dispositivo, tempo médio por imagem, parâmetros, linhas por imagem, % de imagens sem linhas e a **distribuição do número de linhas** (0 a 4).
* **Concordância** entre os dois, nas imagens registadas:
  * % de imagens com o **mesmo número de linhas** e a diferença média;
  * % das linhas de cada modelo que **o outro também encontrou** (duas linhas são "a mesma" quando a distância média entre elas, em x, na parte em que as duas existem, é de 30 px ou menos, na imagem que os modelos receberam: `LIMIAR_PX` no `comparacao.py`);
  * a **distância média** entre as linhas correspondentes;
  * % de imagens em que **só um** dos dois encontrou linhas.

  Não é uma medida de qualidade (não há linhas de referência marcadas à mão): mostra onde os modelos concordam e onde diferem; os instantes ajudam a ver quem tem razão.
* **Instantes guardados:** a lista (tempo, linhas de cada modelo, trecho), com um botão para abrir a pasta de cada um.

## 7. Treino dos modelos, métricas e perspetiva do vídeo
Os dois modelos só funcionam bem em imagens parecidas com as do treino. Esta secção diz em que foram treinados, como foram avaliados e como deve ser o vídeo (e os cortes) para se aproximar dessas condições.

### 7.1 Conjunto de treino: CULane
Os dois modelos foram treinados e avaliados no mesmo conjunto, o [CULane](https://xingangpan.github.io/projects/CULane):
* imagens de **1640x590 px** (proporção 2,78:1, muito mais "larga" do que os 4:3 das Astra), de câmaras **montadas em seis veículos**, viradas para a frente, em Pequim (mais de 55 horas de vídeo);
* 88 880 imagens de treino, 9 675 de validação e 34 680 de teste; o teste está dividido em situação **normal** e **8 situações difíceis** (multidão, encandeamento, sombras, sem linhas, setas, curvas, cruzamentos e noite);
* marcadas até **4 linhas** por imagem: as das duas faixas junto ao veículo (é por isso que os dois modelos dão no máximo 4).

**Métrica (F1@50):** cada linha, prevista ou marcada, é desenhada com **30 px de largura** na imagem de 1640x590; uma linha prevista conta como certa se a sobreposição (IoU) com uma marcada for **maior do que 0,5**. Com as certas, as erradas e as que faltam calcula-se o F1 (média harmónica de precisão e cobertura). Em "cruzamento" não há linhas para encontrar, por isso conta-se só o número de **falsos positivos** (menos é melhor).

### 7.2 Resultados publicados (CULane, teste)
F1@50 (%) por situação, dos artigos de cada modelo (ResNet34 nos dois):

| Situação | CLRNet ResNet34 | UFLDv2 ResNet34 | Diferença |
|---|---|---|---|
| Normal | 93,49 | 92,5 | +1,0 |
| Multidão (*crowded*) | 78,06 | 74,9 | +3,2 |
| Encandeamento (*dazzle*) | 74,57 | 65,7 | **+8,9** |
| Sombras (*shadow*) | 79,92 | 75,3 | +4,6 |
| Sem linhas (*no line*) | 54,01 | 49,0 | +5,0 |
| Setas (*arrow*) | 90,59 | 88,5 | +2,1 |
| Curvas (*curve*) | 72,77 | 70,2 | +2,6 |
| Cruzamento (falsos positivos, menos é melhor) | 1216 | 1864 | **−648** |
| Noite (*night*) | 75,02 | 70,6 | +4,4 |
| **Total (F1@50)** | **79,73** | **75,9** (76,0 na versão final) | +3,8 |

* **Modelos usados aqui:** o CLRNet do projeto tem os pesos do UnLanedet, com **F1 78,99** no total (o UnLanedet não publica valores por situação; os da tabela são os do artigo do CLRNet, com 79,73). O UFLDv2 é o ONNX do PINTO feito com os pesos oficiais (**F1 76,0** no README do UFLDv2).
* **O que se espera ver na comparação:** o CLRNet é melhor em todas as situações. As maiores diferenças estão no **encandeamento**, nas **sombras**, na **noite**, nas imagens **sem linhas pintadas** e nos **falsos positivos em cruzamentos**. Estes são os casos que vale a pena procurar e guardar com a pausa (secção 4).
* **Velocidade publicada:** CLRNet ResNet34 103 imagens/s (GTX 1080 Ti, artigo do CLRNet); UFLDv2 ResNet34 165 imagens/s (RTX 3090, citado no artigo do CLRKDNet). As GPUs são diferentes, por isso os números não são comparáveis entre si; a comparação na mesma GPU é a da secção 8.
* **A concordância do resumo (secção 6) não é o F1 do CULane:** não há linhas marcadas à mão nos nossos vídeos; mede só se os dois modelos encontram as mesmas linhas, com uma tolerância de 30 px numa imagem de 640 px de largura.

Fontes: CULane ([página do conjunto](https://xingangpan.github.io/projects/CULane)); CLRNet ([Zheng et al., CVPR 2022](https://arxiv.org/abs/2203.10350), Tabela 1); UFLDv2 ([Qin et al., TPAMI 2022](https://arxiv.org/abs/2206.07389), e [README](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)); largura de 30 px da métrica ([CLRerNet](https://arxiv.org/abs/2305.08366)); velocidade do UFLDv2 ([CLRKDNet](https://arxiv.org/abs/2405.12503), Tabela I); CLRNet do UnLanedet ([Model Zoo](https://github.com/zkyntu/UnLanedet)).

### 7.3 Como funciona o UFLDv2 aqui
* **Ideia do UFLD:** em vez de procurar a linha em cada píxel, escolhe, para cada uma de 72 linhas horizontais da imagem (âncoras), em que coluna (de 200) passa cada uma de 4 linhas da estrada; as duas linhas de fora usam 81 âncoras verticais. Por isso as linhas saem com pequenos "degraus" (mais onduladas do que as do CLRNet, que prevê cada linha inteira de uma vez).
* **Pré-processamento** (igual ao demo oficial, `demo.py` e `configs/culane_res34.py`): RGB; corta-se o topo da imagem (40%: o modelo foi treinado a ver só os 60% de baixo, `crop_ratio = 0.6`); redimensiona-se para 1600 x 320; normaliza-se com a média e o desvio do ImageNet.
* **Pós-processamento** (o `pred2coords` oficial, em numpy): uma linha do meio conta se existir em **mais de metade** das 72 âncoras; uma de fora, em **mais de um quarto** das 81; a posição em cada âncora é a média pesada (softmax) da célula mais provável e das vizinhas. As linhas saem da esquerda para a direita, em píxeis da imagem recebida (a conversão foi testada: nas linhas do meio, erro médio de 0,1 px e máximo de 0,5 px).
* **Diferenças em relação ao CLRNet:** o UFLDv2 não dá uma confiança por linha (só "existe/não existe" em cada âncora), por isso não há limiar equivalente ao `--conf-linhas`. Ficheiros: `comparacao_linhas/ufld.py` (detetor), `comparacao_linhas/modelos/ufld.json` (configuração), `comparacao_linhas/modelos/ufld.onnx` (secção 1).

### 7.4 Perspetiva do vídeo e cortes
**Como deve ser o vídeo** (as condições do CULane; README 5.1.5 para o CLRNet, que valem também para o UFLDv2):
* câmara **no veículo, virada para a frente**, ao centro, à altura de um carro, **nivelada** (sem rodar) e alinhada com a estrada;
* **horizonte perto dos 40 a 46% da altura** da imagem, a contar de cima (os cortes de treino dos dois modelos estão nessa zona; o CULane não publica a altura exata do horizonte);
* pouco capô à vista: a parte de baixo da imagem tapada pelo carro não tem linhas e reduz a zona útil;
* estrada com linhas pintadas no sentido da marcha; de dia, sem encandeamento, é onde os dois modelos são mais fiáveis (7.2).

**O que o corte do topo corrige:** os dois modelos ignoram a parte de cima da imagem e foram treinados com o horizonte a uma certa altura: o corte do CLRNet (0,46, `cut_height` 270 de 590) fica aproximadamente no horizonte típico do CULane, e o do UFLDv2 (0,40) um pouco acima. Se o horizonte da câmara estiver noutra altura, o modelo recebe a estrada mais acima ou mais abaixo do que no treino (linhas curtas, tortas ao longe, ou falhadas). As linhas desenhadas estão sempre onde o modelo as deteta (a conversão de coordenadas é exata para qualquer corte); o que muda é **a qualidade da deteção**. Para o ajustar:
1. pause num troço reto e plano; no `original.png` (ou `corrigida.png`) desse instante, veja a altura `y` (em píxeis) do ponto de fuga das linhas e divida pela altura da imagem: é a posição do horizonte, `h` (por exemplo, `y = 230` numa imagem de 480 px dá `h = 0,48`);
2. mantenha a relação de cada modelo com o horizonte: `--corte-linhas` perto de `h` e `--corte-ufld` cerca de 0,06 acima (`h − 0,06`):
   ```bash
   python comparacao_linhas/comparar_linhas.py <entrada> --corte-linhas 0.48 --corte-ufld 0.42
   ```
3. compare os `comparacao.png` do mesmo instante com os cortes por omissão e com os ajustados.

**O que o corte do topo não corrige: a proporção da imagem.** O CULane tem 2,78:1 e a Astra 4:3. Ao passar para a entrada fixa de cada modelo (CLRNet 800x320, UFLDv2 1600x320), a imagem da Astra fica, em relação às do treino, cerca de **2 vezes mais esticada na horizontal**: as linhas aparecem mais "abertas" (menos inclinadas) do que o modelo viu no treino. Isto acontece **por igual aos dois modelos** (a diferença depende só da proporção da imagem: 2,78 ÷ 1,33 ≈ 2,1 nas Astra; ≈ 1,6 em vídeos 16:9, como o nuScenes), por isso a comparação continua justa, mas os dois ficam abaixo dos valores publicados. Cortar só o topo não muda a proporção da parte que os modelos veem; para a aproximar do CULane seria preciso cortar também em baixo (uma faixa horizontal da imagem com a proporção do CULane, à volta do horizonte), o que perde a estrada mais perto do carro.

## 8. Desempenho e GPU
* A GPU é escolhida como no resto do projeto (README 3.5), pelo `gpu.py` do pacote `va_g05`: CUDA se o `onnxruntime-gpu` estiver instalado, senão CPU. No arranque, cada modelo corre uma vez numa imagem neutra, e o dispositivo indicado (terminal, registo e resumo) é o que fica **de facto** em uso; se a GPU falhar, aparece `Aviso: a GPU (CUDA) falhou ao correr o modelo de ...; a usar o CPU`.
* No Windows, as bibliotecas do cuDNN instaladas pelo `pip` são encontradas sozinhas (o `gpu.py` junta as pastas `venv\Lib\site-packages\nvidia\*\bin` ao `PATH` do processo). Antes disso, aparecia `Could not locate cudnn_engines_tensor_ir64_9.dll` e os dois modelos corriam no CPU.
* Tempos medidos no PC do projeto, nuScenes `CAM_BACK` reduzida a 640x360, tempo só do modelo:

  | | CLRNet ResNet34 | UFLDv2 ResNet34 |
  |---|---|---|
  | **GPU (CUDA)** | cerca de 22 ms | cerca de 46 ms |
  | **CPU** | cerca de 184 ms | cerca de 296 ms |

  (GPU: valores do painel, que mostra a média recente; CPU: média de uma sessão de 117 imagens.)
* **Nestas condições, o UFLDv2 é mais lento do que o CLRNet** (cerca de 2 vezes em GPU e 1,6 vezes em CPU): a versão CULane ResNet34 recebe uma imagem de 1600x320 (o dobro da do CLRNet, 800x320) e tem uma última camada muito grande. A rapidez anunciada do UFLD é a de outras versões (backbones e entradas menores) e de outras GPUs. Como o UFLDv2 também tem resultados publicados mais baixos em todas as situações (7.2), para a parte 2 não há, à partida, razão para trocar o CLRNet; a comparação serve para o confirmar nas imagens das Astra e para mostrar as diferenças com casos concretos (secção 4).
* Os valores de cada sessão ficam no painel, no terminal, no `ensaio.json` e no resumo.
* Os ms mostrados são só do modelo (sem leitura, correção nem desenho), para comparar os dois de forma justa. Em GPU, a primeira deteção (mais lenta, de preparação) é feita no arranque e não entra nas médias.

## 9. Coerência entre vídeo, deteções e imagens
O que garante que o vídeo original, as deteções e os PNG são do mesmo instante e da mesma imagem:
* **Uma imagem, dois modelos:** em cada instante, a imagem é lida e corrigida **uma vez** e a mesma imagem é passada aos dois modelos; os desenhos são feitos em cópias.
* **O mesmo instante em todo o lado:** o `stamp_ns` (o instante da imagem no bag) é o mesmo nas duas mensagens do registo, no `deteccoes.json` do instante e no nome da pasta (em segundos desde o início).
* **PNG = imagem do bag:** o `original.png` é a imagem do bag com esse `stamp_ns`, sem alterações; o `corrigida.png` é essa imagem corrigida com a calibração guardada no `ensaio.json`; as coordenadas das deteções referem-se à `imagem_dos_modelos` indicada no `deteccoes.json`.
* **Deteções do instante = deteções do registo:** num instante dentro de um trecho, as deteções do `deteccoes.json` são as mesmas que ficam no `deteccoes.mcap` com esse `stamp_ns`.
* **`comparacao.png` = `clrnet.png` | `ufld.png`:** o print lado a lado é feito com as duas vistas guardadas.

Isto foi verificado nos testes: para cada instante, o `original.png` é igual, píxel a píxel, à imagem do bag com o mesmo `stamp_ns`, o `corrigida.png` é igual à sua correção, as deteções do `deteccoes.json` são iguais às do registo e o `comparacao.png` é igual às duas vistas juntas.

## 10. Referência rápida (opções e teclas)

| Comando | Opção | O que faz (valor por omissão) |
|---|---|---|
| `python comparacao_linhas/preparar_ufld.py` | (nenhuma) | descarrega, extrai e verifica o `comparacao_linhas/modelos/ufld.onnx` |
| | `--de <resources.tar.gz ou .onnx>` | usa um ficheiro já descarregado |
| | `--verificar` | só verifica o `comparacao_linhas/modelos/ufld.onnx` |
| `python comparacao_linhas/comparar_linhas.py <entrada>` | `<entrada>` | bag `.mcap`, vídeo ou pasta de imagens (secção 2) |
| | `--camara 5` / `--camara 8` | câmara do bag (a primeira do bag) |
| | `--calib <yaml>` | outra calibração (bags; `src/va_g05/config/camera_astra_<N>.yaml` ou o `/camera_info` do bag) |
| | `--sem-correcao` | não corrigir a distorção (bags) |
| | `--conf-linhas <0-1>` | CLRNet: confiança mínima de uma linha (0.4) |
| | `--corte-linhas <0-1>` | CLRNet: fração cortada no topo (0.46) |
| | `--corte-ufld <0-1>` | UFLDv2: fração cortada no topo (0.4) |
| | `--fps <n>` | vídeos e pastas: imagens por segundo (a do vídeo; pastas: o nome em ns ou 30) |
| | `--largura <px>` | vídeos e pastas: largura (640; `0` não reduz) |
| | `--sem-interface` | sem janela; teclas no terminal |
| | `--trechos "10-20,60-"` | trechos do registo em segundos (desliga a tecla R) |
| | `--registar-tudo` | registar o vídeo todo (`--trechos 0-`) |
| | `--pasta-registo <pasta>` | pasta das sessões (`data/comparacao`) |
| | `--sem-resumo` | não abrir o resumo no fim |
| `python comparacao_linhas/resumo_comparacao.py` | `[<sessão>]` | resumo de uma sessão (a mais recente) |
| | `--pasta-registo <pasta>` | onde procurar a mais recente (`data/comparacao`) |

| Tecla | Janela | Terminal (`--sem-interface`) |
|---|---|---|
| **Espaço** | pausa e guarda o instante / retoma | igual |
| **R** | começa / termina um trecho (também na pausa) | igual |
| **C** | liga / desliga a correção da distorção (bags) | — |
| **Q** / **Esc** | termina; no ecrã "Fim do vídeo", fecha | **Q** termina |

## 11. Ficheiros e manutenção do ramo
**Ficheiros deste ramo**, todos na pasta `comparacao_linhas/` (mais este documento, uma nota no README e três linhas no `.gitignore`):
```
comparacao_linhas/
    comparar_linhas.py      a comparação (secção 3)
    resumo_comparacao.py    o resumo (secção 6)
    comparacao.py           registo, leitura, concordância e instantes
    ufld.py                 o detetor UFLDv2 (secção 7.3)
    preparar_ufld.py        descarrega e verifica o modelo (secção 1)
    modelos/ufld.json       configuração do UFLDv2 (no git); modelos/ufld.onnx (fora do git)
```
Usam, sem alterações, os módulos do projeto, a partir de `src/va_g05` (o pacote `va_g05`, como o offline): `linhas.py`, `gpu.py`, `leitura_bag.py`, `registo.py`, `visual.py`, `gui_comum.py` e `caminhos.py` (onde estão a calibração e a pasta `data/`), e o `offline/teclado.py`. As sessões ficam em `data/comparacao/` (a pasta `data/` do repositório). O UFLDv2 não entra na imagem Docker do online (o `.dockerignore` só deixa entrar o `src/`).

**Manutenção:**
* O ramo **não se junta ao `main`**. Para trazer para aqui correções feitas no `main` (por exemplo, nos módulos acima):
  ```bash
  git switch Extra-Line-Comparison
  git merge main
  git push
  ```
* Para voltar ao trabalho normal: `git switch main`. O `comparacao_linhas/modelos/ufld.onnx` e o `data/comparacao/` não estão no git e ficam na pasta ao mudar de ramo (as sessões antigas, em `data_comparacao/`, podem ser movidas para `data/comparacao/`).

**Créditos:** [Ultra-Fast-Lane-Detection-v2](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2), Qin et al., *Ultra Fast Deep Lane Detection with Hybrid Anchor Driven Ordinal Classification*, TPAMI 2022 (licença MIT); conversão para ONNX: [PINTO_model_zoo](https://github.com/PINTO0309/PINTO_model_zoo/tree/main/324_Ultra-Fast-Lane-Detection-v2) (PINTO0309).
