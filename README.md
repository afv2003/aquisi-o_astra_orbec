# Aquisição e Processamento de Imagem em ROS 2 (Jazzy)

Sistema em Docker para aquisição de imagem com duas câmaras Orbbec Astra (5 e 8) em ROS 2 Jazzy, visualização em tempo real no RViz2 e gravação automática em formato ROS bag (`.mcap`), acompanhado de scripts em Python/OpenCV para processamento offline: correção da distorção da lente, deteção de objetos (YOLO) e deteção de linhas da estrada (CLRNet).

| Secção | O quê | Onde |
|---|---|---|
| 1 | Aquisição (gravar bags) | Linux, com Docker |
| 2 | Processamento offline dos bags | Windows, Linux ou macOS, com `venv` |
| 3 | Calibração das câmaras | Windows |
| 4 | Deteção de linhas da estrada (CLRNet, modelo já incluído) | dentro do processamento |
| 5 | Deteção de objetos (YOLO, modelo já incluído) | dentro do processamento |

## Pré-requisitos
* Aquisição: Linux com Docker e Docker Compose instalados
* Processamento e calibração: Python 3 (com suporte a `venv`)

---

## 1. Aquisição de Dados (No Veículo)

O Docker lança as **duas câmaras Astra (5 e 8) em simultâneo**, cada uma com o seu driver `usb_cam`, abre o `rviz2` e grava as duas num só bag (`.mcap`) na pasta `./data`. Cada câmara publica no seu tópico: `/camara5/image_raw` e `/camara8/image_raw`.

**Ficheiros e como funcionam:**
* `Dockerfile`: imagem com o ROS 2 Jazzy, o driver `usb_cam` e o `rviz2`. Assim, a aquisição não precisa do ROS instalado no PC.
* `compose.yaml`: lança 4 contentores ROS (nós) na mesma rede: um driver por câmara (`driver_camara5` e `driver_camara8`, cada um com o seu namespace e, por isso, o seu tópico), o `rviz` (visualização) e o `gravador_bag` (`ros2 bag record` dos dois tópicos para um só `.mcap`, com o mesmo relógio). Os drivers correm em modo `privileged` para terem acesso aos `/dev/video` do PC.
* `gravar.sh`: associa cada câmara física (5 ou 8) ao seu `/dev/video`, confirma que o dispositivo existe e lança o `docker compose up`. Existe porque a ordem dos `/dev/video` muda de dia para dia; sem ele, as câmaras podiam ficar trocadas nos tópicos.

### Passo 1: Construir a imagem do Docker (Apenas na 1.ª vez)
Antes de utilizar o sistema pela primeira vez, é necessário descarregar e construir a imagem Docker com o ROS 2 e os pacotes da câmara. Abra o terminal na pasta do projeto e execute:
```bash
docker compose build
```

### Passo 2: Ligar as câmaras e ver em que `/dev/video` está cada uma
1. Ligue as duas câmaras ao computador **antes** de iniciar a aquisição.
2. Veja os dispositivos:
   ```bash
   ls /dev/video*
   ```
   Normalmente `/dev/video0` e `/dev/video1` são a webcam do portátil, e as duas Astra ficam em **`/dev/video2`** e **`/dev/video4`** (cada câmara cria dois dispositivos; usa-se o primeiro de cada).
3. O Linux numera as câmaras pela ordem em que as deteta, por isso a mesma câmara pode ficar no `video2` num dia e no `video4` noutro. Para saber qual é qual, desligue uma câmara e veja qual dos dois desaparece.

### Passo 3: Autorizar o uso do ecrã (Apenas uma vez depois de ligar o PC)
Antes de iniciar o Docker, dê permissão para que o RViz consiga abrir a janela gráfica. Só precisa de executar este comando uma vez sempre que liga o computador:
```bash
xhost +local:root
```

### Passo 4: Iniciar a aquisição e gravação
Indique as câmaras **pela ordem dos dispositivos**: a primeira fica no `/dev/video2`, a segunda no `/dev/video4`.
```bash
bash gravar.sh 5 8     # câmara 5 em /dev/video2, câmara 8 em /dev/video4
bash gravar.sh 8 5     # se estiverem ao contrário
bash gravar.sh 5       # só uma câmara (a 5, em /dev/video2)
```
* O script mostra a associação feita (`Câmara 5  ->  /dev/video2`) e lança o `docker compose up`.
* As câmaras ligam, a janela do **RViz2** abre e a gravação começa de imediato dentro da pasta `data/`.
* No RViz2, adicione dois displays **Image** (botão *Add*) com os tópicos `/camara5/image_raw` e `/camara8/image_raw`.
* **Confirme sempre que as câmaras não estão trocadas:** tape a lente da câmara 5 e veja que a imagem que fica preta é a do `/camara5/image_raw`. Se não for, pare (`Ctrl + C`) e lance com a ordem contrária.
* Se um `/dev/video` não existir, o script para e avisa. O `docker compose up` sozinho não funciona: use sempre o `gravar.sh`.
* As duas câmaras juntas usam muita largura de banda USB. Ligue-as em portas de lados diferentes do portátil. Se uma delas não arrancar com erro `No space left on device`, é esse o problema.
* Cada minuto de gravação com as duas câmaras ocupa cerca de 2 GB; confirme que há espaço livre no disco.

### Passo 5: Terminar e guardar a gravação
1. Quando quiser parar de gravar, vá ao terminal onde correu o comando anterior e prima **`Ctrl + C`** (apenas uma vez).
2. Aguarde alguns segundos até que o Docker feche todos os processos em segurança.
3. Os dados gravados ficarão automaticamente disponíveis no seu computador dentro de uma subpasta com a data e hora da recolha (exemplo: `data/viagem_AAAAMMDD_HHMMSS/`). O bag tem as duas câmaras, cada uma no seu tópico.

---

## 2. Processamento de Imagem / Extração (Offline)

O tratamento das imagens com OpenCV ou a extração para vídeo é feito diretamente no computador através de um ambiente virtual Python (`venv`), sem necessidade de ligar a câmara ou o Docker. Funciona em **Linux e em Windows**; cada sistema precisa do seu próprio `venv` (um `venv` criado num sistema não funciona no outro).

### Passo 1: Criar o ambiente virtual `venv` (Apenas na 1.ª vez)
Quando descarregar o repositório pela primeira vez, abra o terminal na pasta do projeto e execute os comandos abaixo para criar o ambiente isolado e instalar as bibliotecas do `requirements.txt` (`opencv-python`, `numpy`, `mcap-ros2-support`, `ultralytics`, `pyyaml` e `onnxruntime`):

**Linux:**
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```
**Windows** (Linha de Comandos):
```
python -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### Passo 2: Como executar o programa Python (Utilização normal)
Sempre que abrir um terminal novo para processar um ficheiro `.mcap` já gravado, siga esta sequência:

1. **Ativar o ambiente virtual:**
   ```bash
   source venv/bin/activate      # Linux
   venv\Scripts\activate         # Windows
   ```
   *(Saberá que está ativo porque aparecerá `(venv)` no início da linha do terminal).*

2. **Verificar o nome do ficheiro `.mcap` gravado (Opcional):**
   Se não souber o nome exato da pasta que o Docker criou, liste os ficheiros `.mcap` existentes com:
   ```bash
   find data/ -name "*.mcap"      # Linux
   dir /s /b data\*.mcap          # Windows
   ```

3. **Confirmar que o bag tem as duas câmaras (Opcional):**
   ```bash
   python -c "from mcap.reader import make_reader; s=make_reader(open('data/<nome_da_pasta>/<nome_do_ficheiro>.mcap','rb')).get_summary(); [print(c.topic, s.statistics.channel_message_counts.get(i,0), 'mensagens') for i,c in s.channels.items()]"
   ```
   Deve aparecer `/camara5/image_raw` e `/camara8/image_raw` com números de mensagens parecidos (cerca de 1800 por minuto, a 30 imagens por segundo). Só funciona em bags completos (com o índice no fim).

4. **Executar o script Python:**
   Corra o programa `processar_bag.py` indicando à frente o caminho completo para o ficheiro `.mcap` que pretende processar:
   ```bash
   python processar_bag.py data/<nome_da_pasta>/<nome_do_ficheiro>.mcap
   ```
   *(Exemplo prático: `python processar_bag.py data/viagem_20261110_101500/viagem_20261110_101500_0.mcap --camara 5`)*
   * Use `--camara 5` ou `--camara 8` para escolher a câmara a processar; cada câmara é processada separadamente.
   * Em cada imagem, por esta ordem, o script:
     1. lê a imagem do tópico da câmara e converte-a para cores (BGR);
     2. corrige a distorção da lente com a calibração dessa câmara (secção 3);
     3. deteta as linhas da estrada na imagem corrigida (CLRNet, secção 4);
     4. deteta objetos na imagem corrigida e desenha as caixas (YOLO, secção 5);
     5. desenha as linhas por cima e mostra tudo na mesma janela, com o tempo, a câmara e o número de linhas.
   * Use `-v` para ver só o vídeo, sem YOLO nem deteção de linhas.
   * Se o `.mcap` estiver vazio (por exemplo, uma cópia que falhou), o script avisa: `não é um ficheiro MCAP válido (0 bytes)`.
   * Se a gravação foi interrompida (Docker parado à força, `Ctrl + C` duas vezes, disco cheio) ou a cópia ficou incompleta, falta o índice que o MCAP guarda no fim do ficheiro. O script avisa (`o bag não tem o índice no fim`), lê as imagens sequencialmente desde o início e para no ponto onde o ficheiro foi cortado; o tempo total aparece como desconhecido.
   * Se parte do ficheiro estiver estragada (por exemplo, numa cópia ou transferência), o script verifica cada bloco de dados do MCAP, salta os blocos estragados (avisa `bloco estragado no bag`) e continua; no fim indica quantos segundos de vídeo faltam. Se isto acontecer, copie de novo o ficheiro original.

5. **Sair do ambiente virtual (Quando terminar o trabalho):**
   ```bash
   deactivate
   ```

### Passo 3: Desempenho (velocidade) e GPU
No vídeo aparece uma linha com o tempo de cada etapa, por exemplo `linhas 340 ms | YOLO 60 ms | 2.5 img/s (gravado a 30)`; no fim, o terminal mostra a média.
* **Em CPU, o processamento é bastante mais lento do que a gravação** (30 imagens por segundo = 33 ms por imagem). A etapa mais pesada é a deteção de linhas (CLRNet, cerca de 100 a 350 ms por imagem, conforme o processador), seguida do YOLO. O vídeo não perde imagens: só passa mais devagar.
* **Com GPU** (medido numa NVIDIA RTX 3050 Ti Laptop): linhas cerca de 37 ms e YOLO cerca de 59 ms por imagem, cerca de 10 imagens por segundo.
* O script espera apenas o que falta para acompanhar o ritmo da gravação; quando o processamento já é mais lento, não acrescenta espera.
* **Para ir mais rápido, use a GPU** (opcional, no `venv`; o `requirements.txt` fica em CPU para funcionar em qualquer PC). Com GPU NVIDIA (testado numa RTX 3050 Ti Laptop):
  * **YOLO:** PyTorch com CUDA 12 (comando de [pytorch.org](https://pytorch.org/get-started/locally/) para o seu sistema):
    ```
    pip uninstall -y torch torchvision
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
    ```
  * **Deteção de linhas:** `onnxruntime-gpu` numa versão compilada com CUDA 12 (anterior à 1.27, que já usa CUDA 13), para usar as mesmas bibliotecas CUDA que o PyTorch acima (não é preciso instalar o CUDA no sistema):
    ```
    pip uninstall -y onnxruntime onnxruntime-directml
    pip install "onnxruntime-gpu<1.27"
    ```
  * Confirme no terminal: `YOLO em: GPU (...)` e `Deteção de linhas: ATIVADA (..., GPU (CUDA), ...)`.
  * Depois de trocar, não volte a correr `pip install -r requirements.txt` (reinstalaria o `onnxruntime` de CPU ao lado do de GPU).
  * Não use o `onnxruntime-directml`: foi testado e falhou com este modelo na RTX 3050 Ti (a GPU deixou de responder).
  * Se a GPU falhar na deteção de linhas, o script avisa e continua em CPU; para voltar tudo ao CPU: `pip uninstall -y onnxruntime-gpu torch torchvision` e `pip install onnxruntime torch torchvision`.

---

## 3. Calibração da câmara (correção da distorção da lente)

A lente da Astra curva ligeiramente as linhas retas, sobretudo nos cantos da imagem. A calibração mede essa distorção uma única vez e o `processar_bag.py` corrige-a em cada imagem, antes da deteção de linhas e do YOLO.

### Passo 1: Gerar a calibração (Windows, uma vez por câmara)
Há duas câmaras Astra, identificadas como **5** e **8**. Cada uma tem a sua calibração, porque duas unidades do mesmo modelo nunca são exatamente iguais.

Num PC com Windows (sem ROS), com a câmara ligada e um xadrez impresso:
```bash
pip install opencv-python numpy
python calibrar_astra.py --camara 5 --cam 1 --colunas 9 --linhas 7 --quadrado 25 --auto
```
* `--camara` é a designação da câmara física (**5** ou **8**), obrigatória.
* `--colunas`/`--linhas` são os **cantos internos** do xadrez (10x8 quadrados → 9x7 cantos).
* `--quadrado` é o lado de um quadrado medido com régua (em mm).
* `--cam` é o número da câmara (`python calibrar_astra.py --listar` mostra as disponíveis).
* Mostre o xadrez em várias posições: centro, cantos, perto, longe, inclinado, até as 4 barras (X, Y, Tam, Incl) ficarem verdes ou haver 40 fotos. É o mesmo critério da ferramenta `camera_calibration` do ROS; antes disso, R, C e Q não ficam disponíveis.
* Tal como no ROS, só são estimados k1 e k2 (k3 = 0) e o xadrez é ignorado quando algum canto fica a menos de 8 px da borda da imagem.
* Na vista corrigida (**U**), o valor `lin.` mostra quanto as linhas do xadrez se afastam de uma reta, em px; quanto mais perto de 0, melhor.
* Depois, por esta ordem:
  1. **R**: desliga a captura automática e abre a revisão com o erro de cada foto.
  2. **O**: remove os outliers. **ENTER** volta à câmara.
  3. **C**: calibra com as fotos que ficaram.
  4. **U** (opcional): mostra a imagem corrigida para verificar.
  5. **Q**: grava e sai.
* Se carregar em **Q** sem ter feito R → O → C, aparece um aviso. Um segundo **Q** grava assim mesmo; outra tecla cancela. **ESC** sai sem gravar.
* O resultado fica em `calibracoes/camara5_<data_hora>/camera_astra_5.yaml` (ou `camara8_...` / `camera_astra_8.yaml`).

**Como funciona o `calibrar_astra.py`:** em cada imagem procura os cantos do xadrez (`findChessboardCornersSB`, refinados com `cornerSubPix`) e, com `--auto`, guarda a foto quando a posição do xadrez é suficientemente diferente das anteriores (critério do ROS). Na calibração (`calibrateCameraExtended`, modelo `plumb_bob`) calcula a matriz da câmara K e os coeficientes de distorção D, com o erro de cada foto. Grava:
* `camera_astra_<N>.yaml`: K e D no formato `camera_info` do ROS (o mesmo da ferramenta `camera_calibration`), lido pelo `processar_bag.py`;
* `calibracao.json`: os mesmos valores mais o relatório (erro RMS, erro por foto, fotos rejeitadas);
* `imagens/`: as fotos usadas, para repetir ou verificar a calibração.

### Passo 2: Copiar a calibração para este projeto
Copie o ficheiro gerado de cada câmara para a pasta `config/` deste repositório:
```
config/camera_astra_5.yaml
config/camera_astra_8.yaml
```

### Passo 3: Usar
O `processar_bag.py` usa automaticamente `config/camera_astra_<N>.yaml` da câmara processada (escolha da câmara: secção 2, Passo 2).
```bash
python processar_bag.py data/<pasta>/<ficheiro>.mcap --camara 5 --calib outro.yaml   # outro ficheiro de calibração
python processar_bag.py data/<pasta>/<ficheiro>.mcap --camara 5 --sem-correcao       # sem correção
```
* A correção é rápida: os mapas de correção são calculados uma vez (`initUndistortRectifyMap`) e aplicados a cada imagem (`remap`).
* Se não houver `config/camera_astra_<N>.yaml`, o script tenta o `/camera_info` gravado no bag; se também não houver, mostra a imagem sem correção e avisa.
* Durante a reprodução, a tecla **C** liga/desliga a correção, para comparar.
* Corra o script a partir da pasta do repositório (o caminho `config/` é relativo a essa pasta).

> **Importante:** a calibração só é válida para a resolução usada (640x480, a resolução por omissão do `usb_cam`). Se mudar a resolução, a focagem ou a lente, repita a calibração.

---

## 4. Deteção de linhas da estrada (CLRNet)

O `processar_bag.py` deteta as linhas da estrada em cada imagem, **sempre** (exceto com `-v`), com o modelo **CLRNet ResNet34** treinado no conjunto de dados CULane, tal como disponibilizado pelo [UnLanedet](https://github.com/zkyntu/UnLanedet). O modelo está guardado em formato ONNX em `modelos/linhas.onnx` e corre com a biblioteca `onnxruntime` (incluída no `requirements.txt`).

> **Para usar a deteção de linhas basta a secção 4.1.** As secções 4.2 e 4.3 explicam como funciona e como o modelo foi gerado (registo do que foi feito, **não é preciso repetir**). A secção 4.4 só é precisa para gerar o modelo de novo (por exemplo, depois de o afinar).

### 4.1 Utilização (para todos)
Não é preciso instalar nada além do `requirements.txt` (secção 2):
```bash
python processar_bag.py <bag>.mcap --camara 5                       # objetos + linhas
python processar_bag.py <bag>.mcap --camara 5 --corte-linhas 0.5    # ignorar a metade de cima da imagem
python processar_bag.py <bag>.mcap --camara 5 --conf-linhas 0.2     # mostrar linhas com menos confiança
python processar_bag.py <bag>.mcap --camara 5 -v                    # só vídeo (sem YOLO nem linhas)
```
* No terminal aparece `Deteção de linhas: ATIVADA (CLRNet ResNet34, ..., CPU, corte do topo 46%, confiança mínima 0.4)` (ou `GPU (CUDA)`); no vídeo, as linhas aparecem coloridas e o contador `LINHAS: N`.
* Funciona em **Windows, Linux e macOS**, com ou sem GPU. Não precisa do UnLanedet, de PyTorch, nem de máquinas virtuais.
* **`--corte-linhas`**: o modelo só analisa a parte de baixo da imagem (no CULane corta-se 46 % em cima). Se as linhas saírem mal posicionadas, experimente valores entre 0.3 e 0.6.
* **`--conf-linhas`**: confiança mínima para mostrar uma linha (0.4 por omissão, o valor usado na avaliação do modelo). Com valores mais baixos (0.2 a 0.3) aparecem linhas que o modelo vê com dúvida, por exemplo em imagens fora das condições da secção 4.5, mas também mais linhas falsas. Abaixo de 0.1 não é útil.
* Velocidade e uso da GPU: secção 2, Passo 3. Se a GPU falhar, a deteção continua em CPU.
* Se o `modelos/linhas.onnx` não existir ou o `onnxruntime` não estiver instalado, o script avisa e continua sem a deteção de linhas.

### 4.2 Como funciona
```
processar_bag.py  ──(cada imagem, já sem distorção)──▶  linhas.py  ──▶  modelos/linhas.onnx (onnxruntime)
                                                          │               + modelos/linhas.json (parâmetros)
                                                          └──▶  linhas desenhadas na imagem
```
O `linhas.py` reproduz o processamento do UnLanedet em cada imagem:
1. **Pré-processamento:** corta o topo da imagem (`corte_topo`), redimensiona para 800×320 e divide por 255 (igual ao treino no CULane).
2. **Rede (ONNX):** devolve 192 propostas de linha, cada uma com 78 valores (confiança, início, ângulo, comprimento e 72 posições x).
3. **Pós-processamento:** fica com as propostas com confiança ≥ 0.4, remove duplicados (NMS: duas linhas a menos de 50 px de distância média são a mesma) e guarda no máximo 4 linhas, convertidas em pontos da imagem original.

No UnLanedet, o NMS é código CUDA compilado (só corre em Linux com GPU). No `linhas.py` foi reescrito em Python/numpy, seguindo o mesmo algoritmo (`unlanedet/layers/ops/csrc/nms_kernel.cu`); foi verificado que dá o mesmo resultado em 300 casos de teste.

### 4.3 Como o modelo foi gerado (registo; não é preciso repetir)
**Porquê ONNX.** O UnLanedet só funciona em Linux, com versões antigas de Python e PyTorch e com código CUDA compilado. Para que qualquer pessoa (incluindo outros grupos, na parte 2) possa processar os bags em qualquer PC, sem máquinas virtuais, o modelo foi **exportado uma única vez** para ONNX, que corre em qualquer sistema só com o `onnxruntime`. Só quem gera o modelo precisa do UnLanedet.

**Que modelo.** CLRNet ResNet34 treinado no CULane (F1 78.99 no UnLanedet), pesos `clrnet_model_best_culane.pth` do [Model Zoo do UnLanedet](https://github.com/zkyntu/UnLanedet). Nota: na tabela do Model Zoo, os links do ResNet34 e do ResNet50 estão trocados; o ficheiro certo foi confirmado pelos parâmetros dentro de cada um. O CLRerNet também pode ser exportado (`--modelo clrernet_r34`); o DiffusionLane não foi usado.

**Ambiente de exportação (`.venv-exportar`).** A exportação foi feita num ambiente Python **separado** do `venv` normal, chamado `.venv-exportar`, que **não vai para o git** (está no `.gitignore`; ocupa vários GB e só funciona no PC onde foi criado). Foi criado com:
* **Python 3.10** (obtido com o `uv`, sem alterar o Python do sistema), porque o UnLanedet exige `numpy==1.23.1`, que não existe para versões de Python mais recentes;
* **PyTorch 2.3.1 para CPU** (a versão do UnLanedet; não é preciso GPU para exportar);
* as bibliotecas do UnLanedet **com versões fixas**, porque o `requirements.txt` dele não fixa versões e as mais recentes já não funcionam com o Python 3.10 nem com o `numpy` 1.23.1;
* o `albumentations==0.4.6` instalado à parte (é antigo: precisa de `setuptools<70`, que ainda tem o `pkg_resources`, e de `--no-build-isolation` e `--no-deps`);
* sem o `albucore` (está no `requirements.txt` do UnLanedet, mas o código não o usa).

**Exportação sem GPU nem compilação.** As partes do UnLanedet escritas em CUDA (o NMS e o DCN) não fazem parte da rede do CLRNet, só do pós-processamento ou de outros modelos. O `exportar_linhas.py` substitui-as por módulos vazios, o que permite carregar o modelo e exportá-lo em qualquer sistema, sem compilar nada.

**Verificação.** O `exportar_linhas.py` confirmou que (1) o ONNX é válido, (2) dá a mesma saída que o PyTorch e (3) dá as mesmas linhas, terminando com `EXPORTAÇÃO VERIFICADA COM SUCESSO`. A exportação foi feita em Windows, sem máquina virtual, em 10/11/2026. O `modelos/linhas.json` regista o modelo, os pesos, a origem e a data da exportação.

### 4.4 Gerar o modelo de novo (só se necessário)
Só é preciso para trocar de modelo, para exportar uma versão afinada com imagens próprias, ou se o `linhas.onnx` se perder. Cada novo `linhas.onnx` acrescenta cerca de 88 MB ao histórico do git, por isso não o regenere sem necessidade.

O código do UnLanedet é clonado **fora** da pasta do repositório (para não ir para o git), e o `.venv-exportar` é criado dentro dela (já está no `.gitignore`).

**Linux (Ubuntu):**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh                 # instalar o uv (uma vez); abrir um terminal novo
cd <pasta que contém o repositório> && git clone https://github.com/zkyntu/UnLanedet
cd <pasta do repositório>
uv venv --seed --python 3.10 .venv-exportar
source .venv-exportar/bin/activate
pip install torch==2.3.1 torchvision==0.18.1 --index-url https://download.pytorch.org/whl/cpu
pip install "setuptools<70" wheel
pip install "numpy==1.23.1" "scipy==1.10.1" "protobuf==4.25.5"
pip install fvcore cloudpickle omegaconf hydra-core "shapely==2.0.6" "scikit-image==0.21.0" "scikit-learn==1.3.2" "imgaug==0.4.0" p_tqdm tensorboard "timm==1.0.9" einops "opencv-python==4.8.1.78" "onnx==1.16.2" "onnxruntime==1.18.1" "numpy==1.23.1" "scipy==1.10.1" "protobuf==4.25.5"
pip install --no-build-isolation --no-deps "albumentations==0.4.6"
python -c "import numpy, scipy, cv2, imgaug, albumentations, torch, onnx, onnxruntime; print('numpy', numpy.__version__, '| torch', torch.__version__, '| tudo ok')"
python exportar_linhas.py --unlanedet ../UnLanedet
deactivate
```

**Windows** (Linha de Comandos; foi o caso testado):
```
pip install uv
cd <pasta que contém o repositório>
git clone https://github.com/zkyntu/UnLanedet
cd <pasta do repositório>
uv venv --seed --python 3.10 .venv-exportar
.venv-exportar\Scripts\activate
pip install torch==2.3.1 torchvision==0.18.1 --index-url https://download.pytorch.org/whl/cpu
pip install "setuptools<70" wheel
pip install "numpy==1.23.1" "scipy==1.10.1" "protobuf==4.25.5"
pip install fvcore cloudpickle omegaconf hydra-core "shapely==2.0.6" "scikit-image==0.21.0" "scikit-learn==1.3.2" "imgaug==0.4.0" p_tqdm tensorboard "timm==1.0.9" einops "opencv-python==4.8.1.78" "onnx==1.16.2" "onnxruntime==1.18.1" "numpy==1.23.1" "scipy==1.10.1" "protobuf==4.25.5"
pip install --no-build-isolation --no-deps "albumentations==0.4.6"
python -c "import numpy, scipy, cv2, imgaug, albumentations, torch, onnx, onnxruntime; print('numpy', numpy.__version__, '| torch', torch.__version__, '| tudo ok')"
python exportar_linhas.py --unlanedet ..\UnLanedet
deactivate
```
* Escreva cada comando **numa só linha** (ao copiar, as quebras de linha podem cortar os comandos).
* O comando `python -c ...` deve mostrar `numpy 1.23.1 | torch 2.3.1+cpu | tudo ok`.
* **Não atualize** as bibliotecas do `.venv-exportar` (`pip install --upgrade`): as versões são antigas de propósito.
* Se a instalação falhar num PC, use o WSL2 (`wsl --install` no PowerShell como administrador) e siga os passos de Linux lá dentro.

**O que o `exportar_linhas.py` faz:**
1. descarrega os pesos para `pesos/` (não vão para o git; se já lá estiverem, não volta a descarregar);
2. carrega o UnLanedet com as partes CUDA substituídas por módulos vazios, monta a rede e carrega os pesos;
3. exporta a rede para `modelos/linhas.onnx` e grava os parâmetros em `modelos/linhas.json`;
4. verifica a exportação (ONNX válido, mesma saída que o PyTorch, mesmas linhas);
5. com `--imagem <ficheiro.png>`, grava `modelos/teste_linhas.png` com as linhas desenhadas (opcional).

Outros modelos: `--modelo clrernet_r34` (CLRerNet) ou `--modelo clrnet_r50`. No fim deve aparecer **`EXPORTAÇÃO VERIFICADA COM SUCESSO`**; depois, fazer commit do `modelos/linhas.onnx` e do `modelos/linhas.json`.

O `.venv-exportar` pode ser mantido para exportações futuras ou apagado (para libertar espaço): os comandos acima voltam a criá-lo.

### 4.5 Condições de utilização e limitações
O modelo foi treinado no CULane: imagens de uma câmara no carro, virada para a frente, a circular em estrada. Só funciona bem em imagens parecidas:
* **Câmara no veículo, virada para a frente** (no sentido da marcha), à altura aproximada de um carro e alinhada com a estrada (não virada para o lado nem muito inclinada).
* **Horizonte a meio da imagem, ou acima:** o modelo ignora a parte de cima da imagem (46% por omissão), onde no CULane está o céu. Se a câmara estiver montada de outra forma, ajuste com `--corte-linhas` (por exemplo, `--corte-linhas 0.35` se o horizonte estiver mais acima).
* **O que deteta:** linhas pintadas **no sentido da estrada** (contínuas ou tracejadas), até 4 por imagem (as faixas junto ao veículo), com confiança de pelo menos 0.4. Quando faltam linhas pintadas, prevê a sua posição provável, mas com menos confiança.
* **O que não deteta:** linhas **atravessadas** (passadeiras, linhas de paragem), setas e outras marcas no chão. Lancis e bermas só aparecem às vezes, com confiança baixa.
* **Exemplo:** num vídeo de teste com a câmara num passeio, baixa e virada na diagonal para uma passadeira, o resultado foi `LINHAS: 0` (a confiança máxima foi 0.27, no lancil). O programa funciona; a imagem é que não corresponde ao treino do modelo.
* A deteção é feita sobre a imagem já corrigida da distorção (secção 3); uma calibração errada também prejudica as linhas.
* Fora destas condições (outra posição da câmara, pistas, pouca luz), é preciso afinar o modelo com imagens próprias e voltar a exportá-lo (secção 4.4).

---

## 5. Deteção de objetos (YOLO)

O `processar_bag.py` deteta objetos em cada imagem, **sempre** (exceto com `-v`), com o modelo **YOLOv8n** da biblioteca [Ultralytics](https://github.com/ultralytics/ultralytics) (`ultralytics`, incluída no `requirements.txt`).

### 5.1 Utilização
Não é preciso fazer nada além da secção 2: o modelo `yolov8n.pt` (cerca de 6,5 MB) já está no repositório.
```bash
python processar_bag.py <bag>.mcap --camara 5        # com YOLO (e linhas)
python processar_bag.py <bag>.mcap --camara 5 -v     # sem YOLO nem linhas
```
* No terminal aparece `A carregar modelo YOLOv8n ...` e `Modo YOLO: ATIVADO`; no vídeo, cada objeto detetado aparece numa caixa com o nome da classe e a confiança.
* Corra o script a partir da pasta do repositório: o modelo é lido de `yolov8n.pt` nessa pasta.
* Se a biblioteca `ultralytics` não estiver instalada, o script para e indica o comando para a instalar.

### 5.2 Como funciona
* **Modelo:** YOLOv8n ("nano"), a versão mais pequena e rápida do YOLOv8, pré-treinada no conjunto de dados COCO. Deteta as **80 classes do COCO**, entre elas `person`, `bicycle`, `car`, `motorcycle`, `bus`, `truck`, `traffic light` e `stop sign`. Foi escolhida a versão mais leve para o vídeo não ficar demasiado lento.
* **Onde entra:** corre sobre a imagem **já corrigida** da distorção (secção 3), depois da deteção de linhas (que precisa da imagem limpa, sem caixas desenhadas).
* **Desenho:** as caixas são desenhadas pela própria biblioteca (`resultados[0].plot()`); as linhas da estrada são desenhadas por cima.
* **CPU ou GPU:** o `ultralytics` usa o PyTorch, instalado com ele. Com o PyTorch normal corre em CPU; com o PyTorch com CUDA usa a GPU NVIDIA (secção 2, Passo 3). O terminal mostra `YOLO em: GPU (...)` ou `CPU`.

### 5.3 Trocar de modelo
Para mais precisão (e menos velocidade), pode trocar `yolov8n.pt` por `yolov8s.pt`, `yolov8m.pt`, etc., na linha `modelo_yolo = YOLO('yolov8n.pt')` do `processar_bag.py`; o `ultralytics` descarrega o novo modelo na primeira utilização.

### 5.4 Condições de utilização e limitações
* **Posição da câmara:** qualquer. O COCO tem fotografias tiradas de todos os ângulos, por isso o YOLO não exige a câmara virada para a frente (ao contrário da deteção de linhas).
* **Parâmetros:** os da biblioteca, por omissão: confiança mínima 0.25 e imagem reduzida para 640 px de lado maior (a das Astra, 640x480, entra sem perda).
* **Tamanho dos objetos:** objetos pequenos ou distantes (menos de cerca de 30 px de altura) ou muito tapados são detetados pior, ou não são detetados.
* **Classes parecidas** podem ser confundidas: por exemplo, uma carrinha pode aparecer como `truck` em vez de `car`.
* **Luz:** pouca luz, contraluz e reflexos pioram a deteção.
* O modelo é usado tal como vem pré-treinado: não foi treinado nem afinado com imagens das câmaras Astra.
* Só deteta as 80 classes do COCO (não deteta, por exemplo, marcas no chão; essas são tratadas pela deteção de linhas, secção 4).
* Os resultados não são gravados: apenas mostrados no vídeo.

---

## Créditos
* CLRNet: Zheng et al., *CLRNet: Cross Layer Refinement Network for Lane Detection*, CVPR 2022.
* Implementação e pesos: [UnLanedet](https://github.com/zkyntu/UnLanedet) (licença Apache 2.0).
* Conjunto de dados de treino das linhas: CULane.
* Deteção de objetos: [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) (licença AGPL-3.0), pré-treinado no COCO.
* Verifique as condições de utilização do CULane, do UnLanedet e do Ultralytics antes de usar os modelos fora do contexto académico.
