# Aquisição e Processamento de Imagem em ROS 2 (Jazzy)

Sistema em Docker para aquisição de imagem com duas câmaras Orbbec Astra (5 e 8) em ROS 2 Jazzy, com dois modos de operação: **gravação + processamento offline** (RViz2 e gravação dos vídeos em ROS bag `.mcap`, processados depois em Python/OpenCV, com registo das deteções e janela de resultados) e **processamento online em tempo real** dentro do próprio Docker (com deteção automática de CPU ou GPU NVIDIA, gravação do vídeo original e das deteções, e resumo no fim). Nos dois, as mesmas etapas: correção da distorção da lente e três modelos de deteção, todos em formato ONNX (`onnxruntime`), sem `ultralytics` nem PyTorch: objetos (YOLOv8n), linhas da estrada (CLRNet) e área circulável / estrada (YOLOP).

| Secção | O quê | Onde |
|---|---|---|
| 1 | Aquisição (gravar bags com vídeo bruto) | Linux, com Docker |
| 2 | Processamento offline dos bags | Windows, Linux ou macOS, com `venv` |
| 3 | Calibração das câmaras | Windows |
| 4 | Deteção de linhas da estrada (CLRNet, modelo já incluído) | dentro do processamento |
| 5 | Deteção de objetos (YOLO, modelo já incluído) | dentro do processamento |
| 6 | Deteção da estrada (YOLOP, modelo já incluído) | dentro do processamento |
| 7 | Registo das deteções e vídeos dos trechos registados | dentro do processamento + `reproduzir_registo.py` |
| 8 | Janela de resultados: resumo e percurso no mapa (`gui_resultados.py`) | Windows, Linux ou macOS, com `venv` |
| 9 | Processamento em tempo real / online (câmaras + deteção + gravação do vídeo e das deteções + resumo) | Linux, com Docker (CPU ou GPU automática) |

## Pré-requisitos
* Aquisição (Secção 1) e Processamento em tempo real (Secção 9): Linux com Docker e Docker Compose instalados *(opcional: placa gráfica NVIDIA + NVIDIA Container Toolkit para aceleração por GPU na Secção 9)*
* Processamento offline (Secções 2, 7 e 8) e calibração (Secção 3): Python 3 (com suporte a `venv`)

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

### Passo 6: Copiar os bags para outro PC (com verificação)
Os bags são ficheiros grandes (cerca de 2 GB por minuto) e estragam-se facilmente numa cópia, sem aviso: o ficheiro fica com o tamanho certo, mas com blocos danificados (pontos coloridos na imagem, ou erros a ler o bag). Já aconteceu neste projeto: os bags passaram do Ubuntu para uma pen, depois para o Windows e foram enviados pelo browser do Windows; chegaram estragados, embora o original estivesse bom.
1. **No PC da gravação, logo depois de gravar**, guarde a "impressão digital" de cada bag:
   ```bash
   sha256sum data/viagem_<data_hora>/*.mcap > data/viagem_<data_hora>/sha256.txt
   ```
2. **Envie diretamente a partir do Linux** onde foi gravado (zip e WeTransfer no browser do Ubuntu, `scp`, disco de rede), com o `sha256.txt` dentro da pasta. Evite passos intermédios:
   * **Pen ou disco USB:** ejete sempre antes de tirar (no Linux, `sync` antes de ejetar); se não, parte dos dados pode não ter sido escrita.
   * **Dual boot:** não passe os bags por uma partição NTFS partilhada nem leia a partição do Ubuntu no Windows com programas extra; com o Arranque Rápido do Windows ligado, os ficheiros podem ficar corrompidos.
3. **No PC de destino, antes de processar**, confirme que o código é igual ao do `sha256.txt`:
   ```bash
   sha256sum -c data/viagem_<data_hora>/sha256.txt                              # Linux
   certutil -hashfile data\viagem_<data_hora>\viagem_<data_hora>_0.mcap SHA256   # Windows (comparar à mão)
   ```
   * O "Testar arquivo" do 7-Zip só confirma que o zip chegou igual ao que foi criado; não deteta um ficheiro que já estava estragado antes de entrar no zip. Por isso, a comparação com o `sha256.txt` do PC da gravação é a que conta.

---

## 2. Processamento de Imagem / Extração (Offline)

O tratamento das imagens com OpenCV ou a extração para vídeo é feito diretamente no computador através de um ambiente virtual Python (`venv`), sem necessidade de ligar a câmara ou o Docker. Funciona em **Linux e em Windows**; cada sistema precisa do seu próprio `venv` (um `venv` criado num sistema não funciona no outro).

### Passo 1: Criar o ambiente virtual `venv` (Apenas na 1.ª vez)
Quando descarregar o repositório pela primeira vez, abra o terminal na pasta do projeto e execute os comandos abaixo para criar o ambiente isolado e instalar as bibliotecas do `requirements.txt` (`opencv-python`, `numpy`, `mcap-ros2-support`, `pyyaml`, `pillow`, `customtkinter` e `tkintermapview` para a janela de resultados (secção 8), e o motor `onnxruntime`):

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
* Por omissão, o `requirements.txt` vem configurado para **CPU** (`onnxruntime`), que funciona em qualquer PC.
* **Para usar uma GPU NVIDIA**, o `requirements.txt` já inclui as opções respetivas comentadas (`onnxruntime-gpu[cuda,cudnn]` para drivers com CUDA 13.0+ ou `onnxruntime-gpu[cuda,cudnn]==1.26.0` para drivers com CUDA 12.x). Basta comentar a linha `onnxruntime` e descomentar a opção de GPU certa antes de instalar — veja as instruções no Passo 3.
* **Nunca deixe `onnxruntime` e `onnxruntime-gpu` descomentados ao mesmo tempo** (nem volte a correr `pip install -r requirements.txt` com o `onnxruntime` de CPU ativo num `venv` de GPU): os dois pacotes entram em conflito e o processamento cai silenciosamente para CPU (ver o aviso no Passo 3).

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
   *(Exemplo prático: `python processar_bag.py data/viagem_20261110_101500/viagem_20261110_101500_0.mcap`)*
   * **Câmaras:** por omissão processam-se **todas as câmaras do bag**. Com as duas, o vídeo mostra-as **lado a lado, a 5 à esquerda e a 8 à direita**, cada uma com o seu painel; o registo (secção 7), a reprodução e a janela de resultados (secção 8) também tratam as duas juntas.

     | Opção | Processa |
     |---|---|
     | (nenhuma) | todas as câmaras do bag (as duas, lado a lado, ou a única que tiver) |
     | `--camara 5` / `--camara 8` | só essa câmara (exatamente como antes) |
     | `--camara 5,8` | as duas (o mesmo que sem a opção, num bag com as duas) |

     * Cada câmara usa a sua calibração (`config/camera_astra_5.yaml` e `config/camera_astra_8.yaml`); `--calib` só se usa com uma câmara.
     * As imagens das duas câmaras vêm intercaladas no bag e são processadas pela ordem de gravação, cada uma com os mesmos modelos: **com as duas câmaras, o processamento demora o dobro** (passam o dobro das imagens). Para registar mais depressa, use `--sem-interface` (secção 7) ou desligue os modelos de que não precisa.
     * As teclas (**C**, **O**, **L**, **E**, **V**, **R**, **Espaço**, **Q**) atuam nas duas câmaras ao mesmo tempo. A janela é larga (1284 px com as Astra) e pode ser redimensionada com o rato, mantendo a proporção.
     * Antes de chegar a primeira imagem de uma câmara, o lado dela aparece escuro, com `à espera da primeira imagem`.
     * Os bags antigos, de uma só câmara (tópico `/image_raw`), continuam a funcionar como antes: a câmara lê-se do nome da pasta (`viagem_camara5_...`) ou de `--camara`.
   * Em cada imagem, por esta ordem, o script:
     1. lê a imagem do tópico da câmara e converte-a para cores (BGR);
     2. corrige a distorção da lente com a calibração dessa câmara (secção 3);
     3. deteta as linhas da estrada na imagem corrigida (CLRNet, secção 4);
     4. deteta a estrada (área circulável) na mesma imagem (YOLOP, secção 6);
     5. deteta objetos na imagem corrigida (YOLO, secção 5);
     6. desenha os resultados (estrada, linhas e objetos, por esta ordem) e o painel de informação, e mostra tudo na mesma janela (ver "Informação no vídeo", abaixo).
   * **Ligar e desligar cada deteção** (as opções podem juntar-se, por exemplo `--sem-objetos --sem-estrada`):

     | Deteção | Desligar ao arrancar | Tecla no vídeo | Confiança mínima (por omissão) |
     |---|---|---|---|
     | Objetos (YOLO) | `--sem-objetos` | **O** | `--conf-objetos` (0.25); classes: `--classes-objetos` (as de estrada) |
     | Linhas (CLRNet) | `--sem-linhas` | **L** | `--conf-linhas` (0.4) |
     | Estrada (YOLOP) | `--sem-estrada` | **E** | `--conf-estrada` (classe mais provável) |
     | Todas | `-v` (só vídeo) | **V** (desliga todas / volta a ligá-las) | — |

     * Uma deteção desligada com a tecla deixa também de ser calculada, por isso o vídeo fica mais rápido; no painel, o marcador fica vazio e aparece `desligado`.
     * Uma deteção desligada na linha de comandos nem é carregada (arranque mais rápido), e a sua tecla não faz nada.
     * Outras teclas: **Espaço** pausa, **C** liga/desliga a correção da distorção, **R** começa/termina um trecho do registo (secção 7), **Q** fecha.
   * **Sem janela:** `--sem-interface` processa sem mostrar o vídeo, o mais depressa possível, e mostra o progresso no terminal; serve sobretudo para registar (secção 7).
     * **Confiança mínima:** um número de 0 a 1. Mais alto mostra menos deteções, mas mais seguras; mais baixo mostra mais, incluindo erradas.
   * Se o `.mcap` estiver vazio (por exemplo, uma cópia que falhou), o script avisa: `não é um ficheiro MCAP válido (0 bytes)`.
   * Se a gravação foi interrompida (Docker parado à força, `Ctrl + C` duas vezes, disco cheio) ou a cópia ficou incompleta, falta o índice que o MCAP guarda no fim do ficheiro. O script avisa (`o bag não tem o índice no fim`), lê as imagens sequencialmente desde o início e para no ponto onde o ficheiro foi cortado; o tempo total aparece como desconhecido.
   * Se parte do ficheiro estiver estragada (por exemplo, numa cópia ou transferência), o script verifica cada bloco de dados do MCAP, salta os blocos estragados (avisa `bloco estragado no bag`) e continua; no fim indica quantos segundos de vídeo faltam. Se isto acontecer, copie de novo o ficheiro original e verifique-o (secção 1, Passo 6).
   * **Informação no vídeo:**
     * **Painel** (canto superior esquerdo de cada câmara): tempo decorrido e total, com barra de progresso; câmara e estado da correção (a laranja quando a imagem não está corrigida); uma linha por modelo, identificado pelo nome (`YOLOv8n`, `CLRNet`, `YOLOP`) e por um marcador da sua cor, com o resultado (número de objetos, número de linhas, percentagem da imagem que é área circulável) e o tempo da etapa; no fim, as imagens processadas por segundo e o ritmo da gravação.
     * **Cores fixas**, as mesmas em todas as imagens (definidas no `visual.py`):

       | Modelo (nome no painel) | Deteta | Desenho | Cor |
       |---|---|---|---|
       | `YOLOP` | área circulável | área semitransparente com o contorno a cheio | verde |
       | `CLRNet` | linhas da estrada | todas as linhas, com um contorno escuro fino | amarelo |
       | `YOLOv8n` | objetos | caixa fina com os cantos reforçados e etiqueta `classe confiança%` | uma cor por classe: `car` azul, `truck` índigo, `bus` roxo, `motorcycle` rosa, `bicycle` rosa-claro, `person` vermelho, `traffic light` laranja, `stop sign` laranja-claro |

       As classes de objetos não usam o verde nem o amarelo, para não se confundirem com a estrada e as linhas. Outras classes (com `--classes-objetos todas`) têm também uma cor fixa, escolhida pelo nome.
     * Quando há muitos objetos juntos, as etiquetas que se sobreporiam a outras (ou ao painel) não são escritas; a caixa é sempre desenhada.
     * O texto usa uma fonte do sistema (Segoe UI no Windows; Ubuntu ou DejaVu Sans no Linux), através do `pillow`.

5. **Sair do ambiente virtual (Quando terminar o trabalho):**
   ```bash
   deactivate
   ```

### Passo 3: Desempenho (velocidade) e GPU
No painel do vídeo aparece o tempo de cada etapa (à direita de cada deteção) e, em baixo, as imagens processadas por segundo, por exemplo `1.2 img/s · gravado a 30 img/s`; no fim, o terminal mostra a média.
* **Em CPU, o processamento é bastante mais lento do que a gravação** (30 imagens por segundo = 33 ms por imagem). A etapa mais pesada é a deteção de linhas (CLRNet, cerca de 100 a 350 ms por imagem, conforme o processador), seguida do YOLO. O vídeo não perde imagens: só passa mais devagar.
* **Com GPU** (referência medida anteriormente numa NVIDIA RTX 3050 Ti Laptop, imagens de 640 px de largura, antes da passagem do YOLO para ONNX): objetos cerca de 20 ms, linhas cerca de 37 ms e estrada cerca de 62 ms por imagem; com as três deteções, cerca de 7 a 8 imagens por segundo. Para ir mais depressa, desligue as deteções de que não precisa (tabela do Passo 2).
* O script espera apenas o que falta para acompanhar o ritmo da gravação; quando o processamento já é mais lento, não acrescenta espera.
* **Para ir mais rápido, use a GPU NVIDIA** (opcional, no `venv`; o `requirements.txt` vem em CPU por omissão, com as opções de GPU já preparadas em comentário):
  1. Confirme a versão CUDA suportada pelo driver da sua placa gráfica:
     ```bash
     nvidia-smi
     ```
     Veja o valor de `CUDA Version` no canto superior direito da tabela.
  2. Abra o `requirements.txt`, comente a linha de CPU (`# onnxruntime`) e descomente **apenas uma** das linhas de GPU:
     * **CUDA 13.0 ou superior:** `onnxruntime-gpu[cuda,cudnn]`
     * **CUDA 12.x (drivers mais antigos):** `onnxruntime-gpu[cuda,cudnn]==1.26.0`
     *(O extra `[cuda,cudnn]` instala automaticamente as bibliotecas CUDA e cuDNN via `pip`, sem ser preciso instalar o CUDA no sistema.)*
  3. Se o `venv` ainda não existir, crie-o seguindo o Passo 1. Se já tinha instalado a versão CPU no mesmo `venv`, desinstale primeiro o `onnxruntime` antes de voltar a correr o `pip install -r requirements.txt`:
     ```bash
     pip uninstall -y onnxruntime onnxruntime-directml
     pip install -r requirements.txt
     ```
  4. Confirme no terminal, ao arrancar o `processar_bag.py`: `Deteção de objetos: ATIVADA (YOLOv8n, GPU (CUDA), ...)`, `Deteção de linhas: ATIVADA (..., GPU (CUDA), ...)` e `Deteção da estrada: ATIVADA (..., GPU (CUDA), ...)`.
  > **Atenção:** não deixe `onnxruntime` e `onnxruntime-gpu` ativos ao mesmo tempo no `requirements.txt`, nem corra `pip install -r requirements.txt` num `venv` de GPU depois de um `git pull` que volte a descomentar `onnxruntime`. O `pip` instala a versão de CPU ao lado da versão GPU, os dois partilham os mesmos ficheiros e os modelos passam a correr no CPU sem dar erro (só ficam várias vezes mais lentos).
* **Se ficar lento de repente** (por exemplo, linhas e estrada a mais de 150 ms por imagem com GPU):
  1. Veja no terminal, no arranque, se as três deteções dizem `GPU (CUDA)` ou `CPU`.
  2. Veja que versões do `onnxruntime` estão instaladas: `pip list | findstr onnxruntime` (Windows) ou `pip list | grep onnxruntime` (Linux).
  3. Se aparecerem as duas (`onnxruntime` e `onnxruntime-gpu`), desinstale ambas e reinstale só a versão GPU (desinstalar só a de CPU pode estragar a outra, porque partilham ficheiros):
     ```bash
     pip uninstall -y onnxruntime onnxruntime-gpu
     pip install "onnxruntime-gpu[cuda,cudnn]"          # ou ==1.26.0 se o driver for CUDA 12.x
     ```
  4. Num portátil, ligue o carregador: só com bateria, a GPU fica muito mais lenta.
* Não use o `onnxruntime-directml`: foi testado e falhou com o modelo de linhas na RTX 3050 Ti (a GPU deixou de responder).
* Se a GPU falhar numa deteção, o script avisa e continua em CPU. Para voltar ao CPU de forma permanente: comente a linha `onnxruntime-gpu` no `requirements.txt`, descomente `onnxruntime`, corra `pip uninstall -y onnxruntime-gpu` e depois `pip install -r requirements.txt`.

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
O `processar_bag.py` usa automaticamente `config/camera_astra_<N>.yaml` de cada câmara processada (com as duas câmaras, cada uma com o seu ficheiro; escolha das câmaras: secção 2, Passo 2). `--calib` (outro ficheiro) só se usa com uma câmara.
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

O `processar_bag.py` deteta as linhas da estrada em cada imagem, **sempre** (exceto com `-v` ou `--sem-linhas`), tal como o `processar_tempo_real.py` (secção 9), com o modelo **CLRNet ResNet34** treinado no conjunto de dados CULane, tal como disponibilizado pelo [UnLanedet](https://github.com/zkyntu/UnLanedet). O modelo está guardado em formato ONNX em `modelos/linhas.onnx` e corre com a biblioteca `onnxruntime` (incluída no `requirements.txt`).

> **Para usar a deteção de linhas basta a secção 4.1.** As secções 4.2 e 4.3 explicam como funciona e como o modelo foi gerado (registo do que foi feito, **não é preciso repetir**). A secção 4.4 só é precisa para gerar o modelo de novo (por exemplo, depois de o afinar).

### 4.1 Utilização (para todos)
Não é preciso instalar nada além do `requirements.txt` (secção 2):
```bash
python processar_bag.py <bag>.mcap --camara 5                       # objetos + linhas
python processar_bag.py <bag>.mcap --camara 5 --corte-linhas 0.5    # ignorar a metade de cima da imagem
python processar_bag.py <bag>.mcap --camara 5 --conf-linhas 0.2     # mostrar linhas com menos confiança
python processar_bag.py <bag>.mcap --camara 5 --sem-linhas          # sem as linhas (tecla L no vídeo)
```
* No terminal aparece `Deteção de linhas: ATIVADA (CLRNet ResNet34, ..., CPU, corte do topo 46%, confiança mínima 0.4)` (ou `GPU (CUDA)`); no vídeo, as linhas aparecem a amarelo e o painel mostra quantas foram detetadas.
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

O `processar_bag.py` deteta objetos em cada imagem, **sempre** (exceto com `-v` ou `--sem-objetos`), tal como o `processar_tempo_real.py` (secção 9), com o modelo **YOLOv8n** exportado para ONNX (`modelos/objetos.onnx`). Corre com o mesmo `onnxruntime` usado nas linhas e na estrada — não precisa do `ultralytics` nem de PyTorch no `venv` nem no Docker.

### 5.1 Utilização
Não é preciso instalar nada além do `requirements.txt` (secção 2): o modelo `modelos/objetos.onnx` (cerca de 12,3 MB) já está incluído no repositório.
```bash
python processar_bag.py <bag>.mcap --camara 5                       # com objetos (e linhas e estrada)
python processar_bag.py <bag>.mcap --camara 5 --conf-objetos 0.5    # só objetos com confiança >= 0.5
python processar_bag.py <bag>.mcap --camara 5 --sem-objetos         # sem objetos (tecla O no vídeo)
python processar_bag.py <bag>.mcap --camara 5 --classes-objetos "car,truck,traffic light"   # só estas classes
python processar_bag.py <bag>.mcap --camara 5 --classes-objetos todas                       # as 80 classes
```
* No terminal aparece `Deteção de objetos: ATIVADA (YOLOv8n, GPU (CUDA), confiança mínima 0.25)` (ou `CPU`); no vídeo, cada objeto detetado aparece numa caixa da cor da sua classe, com o nome da classe e a confiança, e o painel mostra quantos foram detetados.
* **Classes detetadas:** por omissão, só **elementos de estrada**: `person`, `bicycle`, `car`, `motorcycle`, `bus`, `truck`, `traffic light` e `stop sign`. O terminal mostra a lista usada (`Classes: ...`).
  * Para mudar a lista de forma permanente, altere `CLASSES_ESTRADA` no `objetos.py` (vale para o offline e para o tempo real, secção 9).
  * Para uma só execução, use `--classes-objetos` com os nomes em inglês do COCO, separados por vírgulas e entre aspas se algum tiver espaços (`"traffic light"`). Um nome errado dá erro e mostra a lista de nomes possíveis.
  * O filtro é aplicado no `objetos.py` antes do NMS: as outras classes são descartadas de imediato e não são desenhadas.
* **`--conf-objetos`**: confiança mínima de um objeto (0.25 por omissão, padrão do YOLOv8). Por exemplo, 0.5 esconde as deteções duvidosas (como `car 0.32` em carros distantes).
* Se o `modelos/objetos.onnx` não existir ou o `onnxruntime` não estiver instalado, o script avisa e continua sem a deteção de objetos.

### 5.2 Como funciona
* **Modelo:** YOLOv8n ("nano"), a versão mais pequena e rápida do YOLOv8, pré-treinada no conjunto de dados COCO. Deteta as **80 classes do COCO**, entre elas `person`, `bicycle`, `car`, `motorcycle`, `bus`, `truck`, `traffic light` e `stop sign`. Foi escolhida a versão mais leve para o vídeo não ficar demasiado lento.
* **Formato e código:** tal como as linhas (`linhas.py`) e a estrada (`estrada.py`), o YOLO corre em formato ONNX (`modelos/objetos.onnx`) através do módulo `objetos.py`, usando apenas `onnxruntime`, `numpy` e `opencv-python`. Isto permite manter o `venv` e o Docker leves, sem precisar de `ultralytics` nem de PyTorch.
* **Pipeline no `objetos.py`:**
  1. **Pré-processamento:** redimensiona mantendo a proporção e completa com cinzento até 640×640 (*letterbox*), converte BGR → RGB e divide por 255.
  2. **Rede (ONNX):** devolve 8400 propostas, cada uma com 84 valores (4 coordenadas da caixa e a pontuação das 80 classes do COCO).
  3. **Pós-processamento:** filtra pela confiança mínima e pelas classes escolhidas, aplica NMS separado por classe (`cv2.dnn.NMSBoxes`) e converte as coordenadas para o tamanho da imagem original. Os nomes das classes são lidos dos metadados gravados no próprio ficheiro ONNX.
* **Onde entra:** corre sobre a imagem **já corrigida** da distorção (secção 3), depois da deteção de linhas e da estrada e ainda antes de se desenhar qualquer coisa na imagem.
* **Desenho:** as caixas e as etiquetas são desenhadas pelo `visual.py`, com uma cor fixa por classe, por cima da estrada e das linhas.
* **CPU ou GPU:** usa o mesmo `onnxruntime` / `onnxruntime-gpu` dos outros modelos (secção 2, Passo 3), com passagem automática para CPU caso a GPU falhe. O terminal mostra `Deteção de objetos: ATIVADA (YOLOv8n, GPU (CUDA), ...)` ou `CPU`.

### 5.3 Trocar de modelo
Para mais precisão (e menos velocidade), pode exportar outra variante do YOLOv8 (`yolov8s.pt`, `yolov8m.pt`, etc.) para ONNX num ambiente temporário (ou separado do `venv` principal) com `ultralytics`, `onnx` e `onnxslim`:
```bash
pip install ultralytics onnx onnxslim
python -c "from ultralytics import YOLO; YOLO('yolov8s.pt').export(format='onnx', imgsz=640, opset=17, simplify=True)"
```
Depois, mova o ficheiro `.onnx` gerado para `modelos/objetos.onnx` (substituindo o anterior). Não é preciso alterar código no `objetos.py`, no `processar_bag.py` nem no `processar_tempo_real.py`.

### 5.4 Condições de utilização e limitações
* **Posição da câmara:** qualquer. O COCO tem fotografias tiradas de todos os ângulos, por isso o YOLO não exige a câmara virada para a frente (ao contrário da deteção de linhas).
* **Parâmetros:** confiança mínima 0.25 (ajustável com `--conf-objetos`) e imagem reduzida para 640 px de lado maior (a das Astra, 640x480, entra sem perda); são os valores por omissão do YOLOv8.
* **Tamanho dos objetos:** objetos pequenos ou distantes (menos de cerca de 30 px de altura) ou muito tapados são detetados pior, ou não são detetados.
* **Classes parecidas** podem ser confundidas: por exemplo, uma carrinha pode aparecer como `truck` em vez de `car`.
* **Luz:** pouca luz, contraluz e reflexos pioram a deteção.
* O modelo é usado tal como vem pré-treinado: não foi treinado nem afinado com imagens das câmaras Astra.
* Só conhece as 80 classes do COCO, e por omissão mostra apenas as de estrada (secção 5.1). Não deteta, por exemplo, sinais de trânsito além do `stop sign`, nem marcas no chão (essas são tratadas pela deteção de linhas, secção 4). Para outros sinais seria preciso outro modelo, treinado com eles.
* Os resultados podem ser gravados: no offline, nos trechos registados (secção 7); no tempo real, sempre (secção 9).

---

## 6. Deteção da estrada (YOLOP)

O `processar_bag.py` deteta em cada imagem a **área circulável** (a estrada por onde o veículo pode seguir) e pinta-a de verde, **sempre** (exceto com `-v` ou `--sem-estrada`), tal como o `processar_tempo_real.py` (secção 9). Usa o **YOLOP** em ONNX (`modelos/estrada.onnx`), com o mesmo `onnxruntime` usado nos objetos e nas linhas: não é preciso instalar nada além do `requirements.txt`.

### 6.1 Utilização
```bash
python processar_bag.py <bag>.mcap --camara 5                  # objetos + linhas + estrada
python processar_bag.py <bag>.mcap --camara 5 --conf-estrada 0.9   # só a estrada de que o modelo tem mais certeza
python processar_bag.py <bag>.mcap --camara 5 --sem-estrada         # sem a estrada (tecla E no vídeo)
```
* No terminal aparece `Deteção da estrada: ATIVADA (YOLOP, área circulável, CPU, classe mais provável)` (ou `GPU (CUDA)`); no vídeo, a estrada fica pintada de verde (com o contorno a cheio) e o painel mostra a percentagem da imagem que é estrada.
* **`--conf-estrada`**: o modelo dá a cada píxel uma pontuação de "estrada" de 0 a 1. Por omissão, um píxel é estrada quando essa pontuação é maior do que a de "não estrada" (como no YOLOP); com `--conf-estrada 0.9`, só quando é pelo menos 0.9. Valores mais altos limpam bordas duvidosas. O modelo dá pontuações muito extremas (quase sempre perto de 0 ou de 1), por isso o efeito é pequeno: num frame de teste, a área passou de 23% (omissão) para 16% da imagem com 0.99, e parte do passeio continuou marcada, porque o modelo "tem a certeza" de que é estrada. Estes erros resolvem-se com a câmara na posição certa (secção 6.3), não com o limiar.
* Na linha de desempenho do vídeo aparece o tempo da etapa (`estrada X ms`). Em CPU, o vídeo fica cerca de duas vezes mais lento; com GPU, a estrada demora cerca de 60 ms por imagem (secção 2, Passo 3). Se não precisar dela, desligue-a com `--sem-estrada` ou a tecla **E**.

### 6.2 Como funciona
* **Modelo:** [YOLOP](https://github.com/hustvl/YOLOP) (licença MIT), treinado no BDD100K (vídeos de condução em cidade e estrada, de dia e de noite). É uma rede com três saídas: objetos, área circulável e linhas. Aqui usa-se **só a área circulável**: os objetos já são detetados pelo YOLOv8 (secção 5) e as linhas pelo CLRNet (secção 4), que é mais preciso nas linhas.
* **Ficheiro:** `modelos/estrada.onnx` é o `weights/yolop-640-640.onnx` publicado pelos autores do YOLOP, copiado sem alterações (entrada 640x640). Não foi preciso exportar nada.
* **Pré-processamento** (igual ao `test_onnx.py` do YOLOP): a imagem é reduzida mantendo a proporção (640x480 fica igual) e completada com cinzento até 640x640; depois, RGB, divide por 255 e normaliza com a média e o desvio do ImageNet.
* **Pós-processamento:** em cada píxel fica a classe mais provável (estrada ou não), retira-se a parte cinzenta e a máscara volta ao tamanho da imagem.
* **Ordem:** corre sobre a imagem já corrigida da distorção e **antes** de se desenhar qualquer coisa (caixas do YOLO, linhas), para os desenhos não confundirem o modelo.
* **Saída usada:** `drive_area_seg`, 2 canais (não estrada, estrada) de 640x640, cada um com uma pontuação de 0 a 1 por píxel.
* **Código:** `estrada.py`, com a mesma estrutura do `linhas.py` e do `objetos.py` (os três modelos partilham o mesmo motor `onnxruntime`: GPU se houver, CPU se não houver ou se a GPU falhar).

### 6.3 Condições de utilização e limitações
* **Câmara no veículo, virada para a frente**, como no BDD100K (câmara no tabliê). Noutras posições o resultado piora: num vídeo de teste, com a câmara num passeio virada na diagonal para uma passadeira, o modelo marcou parte do passeio como estrada.
* **"Área circulável" não é "todo o asfalto":** no BDD100K, é sobretudo a faixa onde o veículo vai e as faixas para onde pode mudar; a faixa do sentido contrário e os lugares de estacionamento normalmente não são marcados.
* Funciona sem linhas pintadas, em cruzamentos e de noite, ao contrário da deteção de linhas; mas não diz em que faixa o veículo vai (isso é a deteção de linhas, secção 4).
* As bordas da área são aproximadas (alguns píxeis), e a máscara pode "falhar" em sombras fortes ou em pisos muito diferentes do habitual (empedrado, terra).
* O modelo é usado tal como vem pré-treinado: não foi treinado nem afinado com imagens das câmaras Astra.

---

## 7. Registo das deteções e vídeos dos trechos registados

O `processar_bag.py` pode guardar as deteções de cada imagem num ficheiro, para serem usadas depois sem voltar a correr os modelos. O formato é **o mesmo do processamento em tempo real** (`processar_tempo_real.py`, secção 9), para os registos offline e online se lerem da mesma forma. A partir de um registo, o `reproduzir_registo.py` refaz o vídeo de qualquer trecho (no ecrã ou em `.mp4`), juntando as imagens do bag original com as deteções guardadas.

### 7.1 Registar
O registo é feito por **trechos**: só se guardam as imagens entre o início e o fim de cada trecho. Pode haver vários trechos na mesma execução; ficam todos no mesmo registo (o mesmo "ensaio").
```bash
python processar_bag.py <bag>.mcap                                   # tecla R: começa/termina cada trecho (as duas câmaras)
python processar_bag.py <bag>.mcap --sem-interface                   # sem janela; tecla R no terminal
python processar_bag.py <bag>.mcap --sem-interface --registar-tudo   # o vídeo todo, sem teclas
python processar_bag.py <bag>.mcap --sem-interface --trechos "10-20,35.5-50,60-"
python processar_bag.py <bag>.mcap --camara 5 --registar-tudo        # só a câmara 5
```
* **Duas câmaras:** um só ensaio para as duas; os trechos são **comuns** (a tecla R ou `--trechos` marcam o mesmo intervalo de tempo nas duas câmaras) e cada câmara tem o seu tópico no `deteccoes.mcap`, como no tempo real.
* **Tecla R** (com ou sem janela): a primeira vez começa um trecho, a seguinte termina-o, e assim sucessivamente. Com janela, aparece no canto superior direito o indicador vermelho `REGISTO · trecho N · tempo`; sem janela, o estado aparece na linha de progresso do terminal. Se o vídeo acabar (ou carregar em Q) com um trecho aberto, o trecho termina na última imagem processada.
* **`--trechos`**: os trechos em segundos desde o início do bag (`60-` = do segundo 60 até ao fim). Dá sempre os mesmos trechos, ao contrário da tecla; com esta opção a tecla R não é usada. `--registar-tudo` é o mesmo que `--trechos 0-`.
* **`--sem-interface`**: sem janela, o processamento não espera pelo ritmo da gravação (vai o mais depressa possível) e não desenha nada, por isso é a forma mais rápida de registar. As teclas são lidas do terminal (**R**, **Espaço**, **Q**). Se a entrada não for um terminal (por exemplo, num contentor sem `-it`), as teclas não funcionam e o script avisa: use `--trechos` ou `--registar-tudo`.
* O que fica registado é o que foi calculado em cada imagem: se desligar um modelo com O/L/E, ou a correção com C, durante um trecho, o registo indica-o imagem a imagem.

### 7.2 Onde fica e formato
Cada execução com pelo menos um trecho cria uma pasta (um **ensaio**):
```
data_offline/<viagem>/camaras5e8_<data_hora>/     (duas câmaras; com uma: camara5_<data_hora>/ ou camara8_...)
    deteccoes.mcap    as deteções de todas as imagens de todos os trechos (de todas as câmaras)
    ensaio.json       bag de origem, câmaras, calibração de cada câmara, modelos e limiares, e a lista de trechos
    trecho_<N>.mp4    (só depois de exportar com o reproduzir_registo.py; com as duas câmaras, lado a lado)
```
* **`deteccoes.mcap`**: um ROS bag (MCAP), como o do tempo real, com uma mensagem por imagem no tópico da sua câmara, `/camara5/processamento/dados_deteccoes` ou `/camara8/processamento/dados_deteccoes`, do tipo `std_msgs/msg/String`, com um JSON:

  | Campo | Conteúdo | Também no tempo real |
  |---|---|---|
  | `objetos` | `[{"classe": "car", "confianca": 0.84, "caixa": [x1, y1, x2, y2]}, ...]` | sim |
  | `linhas` | pontos de cada linha, `[[[x, y], ...], ...]` | sim |
  | `estrada_percentagem` | fração da imagem que é área circulável (0 a 1) | sim |
  | `stamp_ns` | instante da imagem no bag original (ns): é o que liga a deteção à imagem | sim (o `header.stamp` da imagem) |
  | `camara` | câmara da imagem (`"5"` ou `"8"`; também está no tópico) | sim |
  | `trecho` | número do trecho | não |
  | `corrigida` | se a imagem estava corrigida da distorção (as coordenadas são dessa imagem) | sim |
  | `modelos_ativos` | que modelos correram nessa imagem | não |
  | `estrada_contornos`, `estrada_buracos` | contorno da área circulável, para a voltar a desenhar | não |

  Os campos extra não incomodam quem só lê os três primeiros. O instante de cada mensagem no MCAP é também o da imagem original (no tempo real, o instante da mensagem é o do processamento, e o da imagem é o `stamp_ns`).
* **`ensaio.json`**: texto legível, com tudo o que é preciso para refazer os vídeos: caminho do bag; a lista `camaras`, com o tópico da imagem, o tópico das deteções, a calibração usada (K, D) e o tamanho da imagem de cada câmara; nomes e limiares dos modelos; e para cada trecho o início e o fim (em ns e em segundos desde o início do bag) e o número de imagens (total e `imagens_por_camara`). É atualizado sempre que um trecho termina, por isso não se perde se o programa parar. Os registos feitos antes (de uma câmara, sem a lista `camaras`) continuam a ser lidos pelo `reproduzir_registo.py` e pela janela de resultados.
* **Tamanho:** cerca de 3 KB por imagem (cerca de 300 MB por hora de vídeo a 30 imagens por segundo, com os três modelos ligados; o dobro com as duas câmaras).
* A pasta `data_offline/` não vai para o git (está no `.gitignore`), tal como `data/`.

### 7.3 Ver ou exportar um trecho (`reproduzir_registo.py`)
```bash
python reproduzir_registo.py                                     # lista os registos e os seus trechos
python reproduzir_registo.py data_offline/<viagem>/<ensaio>             # mostra todos os trechos, por ordem
python reproduzir_registo.py data_offline/<viagem>/<ensaio> --trecho 2  # só o trecho 2 (pode repetir: --trecho 1 --trecho 3)
python reproduzir_registo.py data_offline/<viagem>/<ensaio> --exportar  # grava trecho_<N>.mp4 na pasta do ensaio
python reproduzir_registo.py data_offline/<viagem>/<ensaio> --exportar --sem-interface   # só grava, sem janela
python reproduzir_registo.py data_offline/<viagem>/<ensaio> --camara 8  # registo de duas câmaras: só a câmara 8
```
* **Duas câmaras:** por omissão, as imagens aparecem lado a lado (5 à esquerda, 8 à direita), como no `processar_bag.py`, e o `.mp4` exportado também (`trecho_<N>.mp4`). Com `--camara 5` ou `--camara 8` mostra-se e exporta-se só essa câmara (`trecho_<N>_camara8.mp4`). O ritmo do vídeo é o da câmara com mais imagens no trecho; a outra câmara mostra sempre a sua imagem mais recente.
* **Como funciona:** lê do bag original só as imagens entre o início e o fim de cada trecho e junta a cada uma as deteções com o mesmo instante (`stamp_ns`). Aplica a mesma correção da distorção (com a calibração guardada no `ensaio.json`) e desenha com o mesmo aspeto do `processar_bag.py` (`visual.py`). Os modelos **não** voltam a correr.
* **Teclas:** **Espaço** pausa, **N** passa ao trecho seguinte, **Q** sai.
* **Velocidade (só na janela):** **1** tempo real, **2** duas vezes mais lento, **3** três vezes mais lento; **-** e **+** abrandam ou aceleram em passos de 0.5x, entre 1x e 3x. A velocidade aparece no canto inferior direito. O vídeo exportado (`--exportar`) fica sempre em tempo real, qualquer que seja a velocidade escolhida.
* **Bag original:** é procurado onde estava no processamento e em `data/<viagem>/`. Se o mudou de sítio, use `--bag <caminho do .mcap>`. O script avisa se o tamanho não for o do bag processado.
* Os vídeos `.mp4` têm o ritmo da gravação (por exemplo, 30 imagens por segundo), qualquer que tenha sido a velocidade do processamento.
* O tempo real (secção 9) grava as deteções no mesmo formato (também um tópico por câmara) e o vídeo original no mesmo bag, mas não tem trechos nem `ensaio.json`: não é para reproduzir com este script. Para relacionar as deteções com as imagens, ver a secção 9 («O que é gravado»).

### 7.4 Juntar dois registos de uma câmara (`juntar_registos.py`)
Se as duas câmaras de um bag forem processadas em sessões diferentes (`--camara 5` numa, `--camara 8` noutra), ficam dois registos separados. O `juntar_registos.py` junta-os num registo de duas câmaras, igual ao que sairia de as processar juntas, para se verem lado a lado no `reproduzir_registo.py` e na janela de resultados (secção 8):
```bash
python juntar_registos.py data_offline/<viagem>/camara5_<data_hora> data_offline/<viagem>/camara8_<data_hora>
```
* Cria `data_offline/<viagem>/camaras5e8_juntado_<data_hora>/` (com `deteccoes.mcap` e `ensaio.json`, como na secção 7.2). Os dois registos originais não são alterados.
* Confirma que os dois registos são do mesmo bag (nome e tamanho) e de câmaras diferentes; se não forem, dá erro e não cria nada. Se os modelos ou limiares forem diferentes, avisa (ficam os dois no `ensaio.json`, em `modelos_por_camara`).
* **Trechos: a união.** Os trechos das duas câmaras que se sobrepõem passam a ser um só trecho, do início mais cedo ao fim mais tarde; um trecho que só uma câmara registou fica como trecho dessa câmara. Cada trecho guarda em que intervalos cada câmara foi registada (`intervalos_por_camara`), e o script mostra-os no fim.
  * Nas partes em que só uma câmara foi registada, o lado da outra aparece **escuro, com a mensagem `Câmara N: sem registo neste intervalo`** (no `reproduzir_registo.py`, no vídeo exportado e na janela de resultados).
  * Nessas partes, os veículos por instante (resumo e cores do percurso) contam só a câmara registada.
* Com `--registar-tudo`, ou o mesmo `--trechos` nas duas sessões, os trechos coincidem e o registo juntado fica igual ao de uma só execução com as duas câmaras. Com a tecla R, os trechos de cada câmara ficam ligeiramente diferentes, e as pontas aparecem só com uma câmara.
* Na listagem do `reproduzir_registo.py` (sem argumentos) e no Resumo da janela («Juntado de»), um registo juntado indica os registos de origem e onde cada câmara foi registada.

---

## 8. Janela de resultados (`gui_resultados.py`)

Uma janela, à parte do processamento, para ver os registos feitos pelo `processar_bag.py` (secção 7):
```bash
python gui_resultados.py                              # registos em data_offline
python gui_resultados.py --pasta-registo outra_pasta  # registos noutra pasta
```
* **Bibliotecas:** `customtkinter` (aspeto da janela) e `tkintermapview` (mapa), no `requirements.txt`. Num `venv` que já existia (sobretudo com a versão GPU, secção 2, Passo 3), instale só estas: `pip install customtkinter tkintermapview`.
* Os registos são lidos de `data_offline`. Se a pasta não existir ou não tiver registos, a janela avisa e a tab **Percurso** fica indisponível.
* **Registos de uma ou de duas câmaras:** a janela adapta-se ao registo escolhido. Com uma câmara, tudo fica como antes. Com duas, aparece a escolha da câmara (abaixo) e, ao escolher as duas, o aspeto muda: tabelas com uma coluna/valor por câmara, vídeo lado a lado com mais espaço, e veículos contados por **instante** (soma das duas câmaras). Na lista, os registos de duas câmaras dizem `câmaras 5 e 8`.

### 8.1 Tab Resumo
* Escolhe-se o registo (por omissão, o mais recente) e o **trecho** (todos, ou um só); tudo o que é mostrado se refere à escolha.
* **Câmara** (só nos registos de duas câmaras): **Ambas**, **Câmara 5** ou **Câmara 8**. Com uma só, o resumo é igual ao de um registo de uma câmara. Com **Ambas**:
  * Vídeo e registo: as duas câmaras, a correção de cada uma e o número de imagens de cada uma;
  * Linhas e área circulável: um valor por câmara (`5: … · 8: …`);
  * Deteções por categoria: colunas com as deteções de cada câmara; média, máximo e % contam as imagens das duas;
  * Veículos: **por instante**, somando as duas câmaras (cada imagem de uma câmara com a imagem da outra mais próxima no tempo, até 0,1 s), e a média de cada câmara.
* **Vídeo e registo:** bag, câmara, duração, correção da distorção, data do processamento, trechos, e o número de imagens e a duração do que está selecionado.
* **Modelos:** limiar de cada modelo, tempo médio por imagem, classes do YOLO e velocidade do processamento.
* **Linhas e área circulável:** linhas por imagem e % de imagens sem linhas (CLRNet); área circulável média, mínima e máxima (YOLOP).
* **Deteções por categoria (YOLO):** para cada classe, deteções, média por imagem, máximo numa imagem, % de imagens com a classe e confiança média. As deteções são somadas imagem a imagem (o mesmo veículo visto em 30 imagens conta 30 vezes): sem seguimento dos objetos, não se sabe quantos veículos diferentes passaram.
* **Veículos por imagem** (com as duas câmaras, **por instante**): média, máximo e a % de imagens (instantes) em cada gama de cores do mapa (tab Percurso).

### 8.2 Tab Percurso
Sem GPS, a ligação entre o vídeo e o mapa faz-se à mão, em três passos por trecho.

0. **Registo, trecho e intervalo:** escolhe-se o registo (da lista, ou noutra pasta com «Outra pasta…»), um trecho ou «Todos os trechos», e o intervalo entre referências (5 s por omissão). Num registo de duas câmaras escolhe-se também as **câmaras**: as duas (vídeo lado a lado e cor pela soma dos veículos das duas) ou só uma (tudo como num registo de uma câmara).
1. **Trajeto** (a forma do caminho): escrevem-se as coordenadas de início (`latitude, longitude`, por exemplo `40.6405, -8.6538`, como o Google Maps as copia) e carrega-se em **Ir**; depois clica-se no mapa ponto a ponto, pela ordem em que foi feito, com pontos suficientes para seguir as curvas. «Desfazer ponto» e «Limpar» corrigem. Arrastar move o mapa; a roda do rato aproxima.
2. **Pontos de referência** (onde estava o carro em instantes fixos): a janela mostra a imagem do vídeo (com as duas câmaras, as duas lado a lado, o que ajuda a reconhecer o sítio) em cada instante múltiplo do intervalo, contado desde o início da gravação (5, 10, 15 s…; num trecho de 12,3 a 31,0 s: 15, 20, 25 e 30 s), e clica-se no mapa onde o carro está. O ponto é encostado ao trajeto e fica marcado com um pequeno círculo branco.
   * **Saltar:** se não reconhecer o sítio nesse instante; os dois intervalos vizinhos juntam-se.
   * **Pontos de curva:** se o trajeto cortar uma esquina, escolha **«Acrescentar curva»** (em cima, ao lado de «Marcar referência») e clique no mapa por onde a rua passa; o ponto entra no troço do trajeto mais perto e as referências são reencostadas. Volte a «Marcar referência» para continuar. O botão direito no mapa («Acrescentar ponto de curva ao trajeto aqui») faz o mesmo.
   * Se o trecho for mais curto do que o intervalo, não há referências a marcar: aparece a primeira imagem do trecho e pode passar ao passo seguinte.
   * Uma referência não pode ficar antes da anterior no trajeto; o início e o fim do trecho ficam ligados ao início e ao fim do trajeto.
3. **Mapa, vídeo e intervalos lado a lado:** à esquerda o trajeto, ao meio o vídeo do trecho (com as deteções registadas; com as duas câmaras, lado a lado, e o vídeo fica com mais largura e o mapa com menos), à direita a tabela de **intervalos** (entre referências): tempo, coordenadas da referência de início, média de veículos e drift, cada linha com o fundo da cor do intervalo e o intervalo em curso a negrito; em baixo, o resumo do trecho (distância, duração, velocidade média, veículos por imagem, referências e drift). **Run** começa e **Parar** pára; com vários trechos, passam seguidos.

**Cores:** cada intervalo é pintado com a cor da **média de veículos por imagem** nesse intervalo (`car`, `motorcycle`, `bicycle`, `bus`, `truck`; pessoas e sinais não contam); com as duas câmaras, da média de veículos **por instante, somando as duas câmaras** (os veículos à volta do carro, dos dois lados). A cor só aparece quando o vídeo passa por esse intervalo.

| Média de veículos (arredondada) | 0 | 1–2 | 3–5 | 6 ou mais |
|---|---|---|---|---|
| Cor | verde | amarelo | laranja | vermelho |

As gamas estão em `GAMAS_VEICULOS`, no `registo.py`, e as cores em `CORES_GAMAS`, no `gui_comum.py` (as mesmas no resumo do tempo real, secção 9); o intervalo por omissão em `INTERVALO_REFERENCIAS_S`, no `gui_resultados.py`.

**Posição e drift:** dentro de cada intervalo, a posição parte da referência de início e avança à velocidade média do trecho (comprimento do trajeto a dividir pela duração), sem nunca passar a referência seguinte. No início do intervalo seguinte, salta para a referência seguinte. O **drift** de cada intervalo é a diferença entre onde a estimativa chegou e a referência seguinte: negativo, a estimativa ficou aquém e o troço que faltava fica **cinzento** no mapa (é o drift visível); positivo, a estimativa chegaria à referência antes do tempo (o carro foi mais devagar do que a média). Não há marcador de posição: só o trajeto pintado.

**Guardar e carregar:** o trajeto e as referências guardam-se em ficheiros separados, cada um no seu passo («Guardar trajeto…» / «Carregar trajeto…» no passo 1; «Guardar referências…» / «Carregar referências…» no passo 2). Por omissão ficam na pasta do ensaio, com os nomes `trajeto_trecho<N>.json` e `referencias_trecho<N>.json`. As referências só se carregam com o mesmo intervalo com que foram marcadas; ao carregar ficheiros de outro trecho ou registo, a janela avisa.

* **Internet:** o mapa (OpenStreetMap) é descarregado da internet. Sem ligação, aparece um aviso em vez do mapa, com um botão para tentar de novo.
* **Sair da tab:** pode-se ir para o Resumo a qualquer momento; ao voltar, tudo recomeça (os ficheiros guardados servem para não repetir os cliques).

---

## 9. Processamento em Tempo Real / Online (Em Docker)

Ao contrário das secções 1 e 2 (onde primeiro se grava o vídeo bruto e depois se processa offline num `venv`), este modo **capta a imagem das câmaras Astra, corrige a distorção da lente e corre os três modelos de deteção em tempo real dentro do próprio Docker**, mostra o vídeo anotado ao vivo (as duas câmaras lado a lado, como no offline) e **grava um ROS bag com o vídeo original e as deteções em JSON**. No fim, mostra um resumo das deteções, como a tab Resumo da janela de resultados do offline (secção 8.1).

Não precisa de ter ROS 2, Python, PyTorch nem `ultralytics` instalados no computador: tudo corre dentro dos contentores Docker, com os mesmos módulos do offline (`objetos.py`, `linhas.py`, `estrada.py`, `visual.py`, `registo.py` e a correção da distorção do `leitura_bag.py`).

| | Offline (secções 2 e 7) | Online (esta secção) |
|---|---|---|
| Modelos, classes e limiares | os mesmos | os mesmos |
| Correção da distorção | `config/camera_astra_<N>.yaml` | a mesma (`--sem-correcao` para desligar) |
| Duas câmaras | lado a lado, 5 à esquerda e 8 à direita | igual; **D** desliga/liga a câmara da direita |
| Deteções gravadas | `deteccoes.mcap` nos trechos registados | no bag da sessão, sempre |
| Vídeo | o bag da aquisição (secção 1) | o vídeo original, no mesmo bag das deteções |
| Resumo | `gui_resultados.py`, tab Resumo (e Percurso) | janela de resumo no fim (só Resumo); não se refaz o vídeo |

### Ficheiros e como funcionam
* `Dockerfile_realtime_inter`: cria uma imagem Docker com ROS 2 Jazzy, o driver `usb_cam`, as bibliotecas do sistema para OpenCV e para a janela do resumo (`python3-tk`) e um ambiente Python (`/opt/venv`) instalado a partir do `requirements.txt` (sem `ultralytics` nem PyTorch; o `tkintermapview`, só do mapa do offline, também não é instalado). Recebe o argumento `ONNX_PKG` (por omissão `onnxruntime`, em CPU) para instalar automaticamente apenas a versão certa para o computador (CPU ou GPU).
* `compose_realtime_inter.yaml`: configuração base do Docker Compose (sempre compatível com CPU) que lança 4 serviços na mesma rede:
  * `driver_camara5` e `driver_camara8`: drivers `usb_cam` que publicam `/camara5/image_raw` e `/camara8/image_raw`;
  * `processador`: corre o `processar_tempo_real.py` dentro do Docker (com a pasta do projeto montada em `/workspace`);
  * `gravador_bag`: grava em `./data_realtime_inter/realtime_<data_hora>/` um bag `.mcap` com o vídeo original e as deteções das câmaras ativas (ver «O que é gravado», abaixo).
* `compose_realtime_gpu.yaml`: ficheiro complementar (apenas algumas linhas) que dá acesso da placa gráfica NVIDIA ao contentor `processador`. **Não precisa de o editar**: é carregado automaticamente pelo `processar_inter.sh` apenas quando o PC tem GPU NVIDIA pronta para Docker.
* `processar_inter.sh`: script que deve usar sempre para arrancar o sistema. Associa as câmaras (5 e/ou 8) aos `/dev/video`, **deteta automaticamente se deve usar GPU NVIDIA ou CPU** (por omissão usa CPU se não houver GPU configurada), aceita as opções `-i` (`--sem-interface`, sem janelas) e `--sem-correcao`, constrói a imagem se houver alterações, lança todos os contentores e, no fim, abre a janela do resumo.
* `processar_tempo_real.py`: nó ROS 2 que recebe as imagens das câmaras, corrige a distorção, corre os três modelos ONNX, desenha com `visual.py`, abre **uma janela com as câmaras lado a lado** (exceto no modo `-i`) e publica:
  * `/camara<N>/processamento/dados_deteccoes` (`std_msgs/String` em JSON, o mesmo formato do registo offline);
  * `/camara<N>/processamento/imagem_anotada` (`sensor_msgs/Image`, imagem BGR com o desenho; só é calculada com janela, ou se alguém subscrever o tópico, por exemplo no RViz);
  * `/processamento/estado_camaras` (que câmaras estão a ser processadas; muda quando se desliga ou liga a câmara da direita).

  No fim grava `resumo_online.json` na pasta da sessão (as estatísticas das deteções).
* `resumo_online.py`: janela do resumo, aberta pelo `processar_inter.sh` no fim (usa o `gui_comum.py`, o mesmo da tab Resumo do offline).
* `alternar_camara.sh`: desliga ou volta a ligar a câmara da direita sem janela (modo `-i`), a partir de outro terminal.

### Passo 0: Preparar a GPU NVIDIA no Docker (Opcional — apenas se tiver placa NVIDIA)
O sistema funciona **em CPU por omissão** em qualquer PC com Linux e Docker — se não tiver placa NVIDIA, salte diretamente para o **Passo 1**.

Se o seu PC tem uma placa gráfica NVIDIA e quer usar aceleração por GPU em tempo real, o Docker precisa do **NVIDIA Container Toolkit** instalado (faz-se apenas uma vez no computador):

1. **Verifique se o PC tem driver NVIDIA e se o Docker já o reconhece:**
   ```bash
   nvidia-smi
   docker info | grep -i nvidia
   ```
   * Se o segundo comando já mostrar `Runtimes: ... nvidia`, está pronto — avance para o **Passo 1**.
   * Se `nvidia-smi` funcionar mas `docker info | grep -i nvidia` não mostrar nada, siga os passos 2 a 4 abaixo.

2. **Adicione o repositório oficial da NVIDIA (Ubuntu/Debian):**
   ```bash
   sudo apt-get update && sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg2
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
   curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
   ```

3. **Instale o toolkit e ative o runtime no Docker:**
   ```bash
   sudo apt-get update
   sudo apt-get install -y nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   ```

4. **Confirme que ficou ativo:**
   ```bash
   docker info | grep -i nvidia
   ```
   Deve aparecer `nvidia` na linha `Runtimes:`. A partir daqui, o `processar_inter.sh` deteta automaticamente a placa NVIDIA e a versão CUDA (CUDA 13+ usa `onnxruntime-gpu[cuda,cudnn]`; CUDA 12.x usa `onnxruntime-gpu[cuda,cudnn]<1.27`).

### Passo 1: Ligar as câmaras e confirmar os `/dev/video`
1. Ligue as câmaras Astra ao computador **antes** de arrancar o script (se ligar as duas, use portas USB de lados diferentes do portátil para evitar falta de largura de banda USB).
2. Confirme que os dispositivos existem:
   ```bash
   ls /dev/video*
   ```
   Por omissão o script usa `/dev/video2` para a 1.ª câmara indicada e `/dev/video4` para a 2.ª. Se as suas câmaras ficarem noutros números, pode indicá-los ao arrancar (exemplo: `DEVS_VIDEO="/dev/video4 /dev/video6" bash processar_inter.sh 5 8`).

### Passo 2: Autorizar o Docker a abrir janelas no ecrã (Uma vez por arranque do PC)
Tal como na Secção 1, dê permissão para que as janelas (vídeo e resumo) consigam abrir no ecrã *(se executar com `-i` para correr sem interface gráfica, este passo não é necessário)*:
```bash
xhost +local:root
```

### Passo 3: Iniciar o processamento em tempo real
Abra o terminal na pasta do projeto e lance o `processar_inter.sh` indicando as câmaras pela ordem dos dispositivos `/dev/video`:
```bash
bash processar_inter.sh 5 8              # câmara 5 em /dev/video2, câmara 8 em /dev/video4
bash processar_inter.sh 8 5              # se estiverem ao contrário
bash processar_inter.sh 5                # apenas a câmara 5 em /dev/video2
bash processar_inter.sh 5 8 -i           # sem interface gráfica (corre apenas no terminal/fundo)
bash processar_inter.sh 5 -i             # apenas a câmara 5, sem interface gráfica
bash processar_inter.sh 5 8 --sem-correcao   # sem corrigir a distorção da lente
USAR_GPU=0 bash processar_inter.sh 5 8   # forçar CPU mesmo num PC com GPU NVIDIA
```
* **Deteção automática de hardware:** logo no início, o terminal indica o que escolheu, por exemplo:
  * `Hardware detetado: GPU NVIDIA (CUDA 13) -> onnxruntime-gpu[cuda,cudnn]`
  * `Hardware detetado: CPU -> onnxruntime`
  * ou `Hardware: GPU NVIDIA encontrada, mas falta o NVIDIA Container Toolkit no Docker -> a usar CPU` (neste caso, siga o Passo 0 se quiser usar a GPU).
* **Na 1.ª vez** (ou quando alterna entre CPU e GPU, ou depois de mudar o `Dockerfile_realtime_inter` ou o `requirements.txt`), o Docker constrói a imagem e descarrega as bibliotecas Python (pode demorar alguns minutos, sobretudo com `onnxruntime-gpu[cuda,cudnn]`). Nas execuções seguintes, usa a cache do Docker e arranca em poucos segundos.
* **Confirme no terminal** que os três modelos carregaram no dispositivo esperado e que a correção da distorção está ativa:
  ```text
  Objetos (YOLOv8n): GPU (CUDA)
  Linhas  (CLRNet):  GPU (CUDA)
  Estrada (YOLOP):   GPU (CUDA)
  Câmara 5: correção da distorção com config/camera_astra_5.yaml
  Câmara 8: correção da distorção com config/camera_astra_8.yaml
  ```
  *(ou `CPU`, se estiver a correr sem GPU; sem o ficheiro de calibração de uma câmara, aparece um aviso e essa câmara não é corrigida).*
* **Janela:** uma só janela (`Tempo Real`) com as câmaras **lado a lado, a 5 à esquerda e a 8 à direita** (qualquer que seja a ordem dos `/dev/video`), com a estrada, as linhas e os objetos desenhados ao vivo e o painel de cada câmara: tempo desde o arranque, estado da correção, resultado e tempo (ms) de cada modelo e **deteções por segundo** dessa câmara. A janela pode ser redimensionada com o rato.
* **Correção da distorção:** como no offline (secção 3), cada imagem é corrigida com o `config/camera_astra_<N>.yaml` da sua câmara **antes** das deteções, por isso as coordenadas das deteções são as da imagem corrigida (campo `corrigida` no JSON). O vídeo gravado no bag é o original, sem correção. Com `--sem-correcao`, as deteções são feitas na imagem original.
* Com a opção `-i` (ou `--sem-interface`), não abre janelas gráficas nem desenha nada (mais rápido), mantendo o processamento, a publicação dos tópicos ROS e a gravação do bag a funcionar normalmente.
* Passados 4 segundos, o `gravador_bag` começa automaticamente a gravar em `data_realtime_inter/realtime_AAAAMMDD_HHMMSS/` (o terminal indica a pasta logo no arranque).

### Desligar temporariamente a câmara da direita (mais deteções por segundo na outra)
Os três modelos correm um de cada vez, para uma câmara e depois para a outra: com as duas câmaras, cada uma é processada metade das vezes. Para aumentar a frequência de deteção numa delas durante um trecho, desligue temporariamente a deteção da câmara da direita (a 8, com as duas câmaras):
* **Com janela:** tecla **D** (a mesma tecla volta a ligá-la). O lado direito da janela fica escuro, com `Câmara 8: deteção desligada`.
* **Sem janela (`-i`):** noutro terminal, na pasta do projeto: `bash alternar_camara.sh` (cada vez que corre, desliga ou volta a ligar).
* O terminal do processamento mostra cada mudança, com o instante (`[00:27.9] Câmara 8: deteção DESLIGADA`).
* **A gravação do vídeo da câmara desligada continua** (só a deteção é que pára): o bag tem o vídeo das duas câmaras o tempo todo, e as deteções da câmara 8 só nos períodos em que esteve ligada. Cada mudança fica também gravada no tópico `/processamento/estado_camaras`, e os períodos em que esteve desligada aparecem no resumo.
* A câmara da esquerda nunca se desliga (há sempre pelo menos uma câmara a ser processada). Com uma só câmara, a tecla não faz nada.

### Passo 4: Terminar, gravação e resumo
1. Para terminar, prima **`Q`** (ou `ESC`) na janela do vídeo ou prima **`Ctrl + C`** (apenas uma vez) no terminal *(no modo `-i`, sem interface, use `Ctrl + C`)*.
2. O processamento e a gravação terminam juntos (o bag acaba quando o processamento acaba). Aguarde alguns segundos para o Docker encerrar os contentores em segurança e gravar o índice final do `.mcap`.
3. **Resumo:** com janela, abre-se a seguir a janela **«Resumo do processamento em tempo real»**, com o mesmo conteúdo da tab Resumo do offline (secção 8.1), sem o percurso:
   * **Sessão:** câmaras, início, duração, correção, imagens processadas e **deteções por segundo** de cada câmara, e os períodos em que a câmara da direita esteve desligada;
   * **Modelos:** limiar, dispositivo (CPU/GPU) e tempo médio de cada modelo;
   * **Linhas e área circulável**, **deteções por categoria** e **veículos** (por instante, somando as duas câmaras), iguais aos do offline;
   * com duas câmaras, escolhe-se **Ambas**, **Câmara 5** ou **Câmara 8**, como no offline. Enquanto a câmara 8 esteve desligada, os veículos por instante contam só a câmara 5.

   Feche a janela do resumo para terminar. **Sem janela (`-i`), o resumo não é mostrado**, mas é gravado na mesma (`resumo_online.json`), tal como o vídeo e as deteções.

### O que é gravado (`data_realtime_inter/realtime_<data_hora>/`)
```
data_realtime_inter/realtime_<data_hora>/
    realtime_<data_hora>_0.mcap   o bag: vídeo original e deteções de cada câmara, e o estado das câmaras
    metadata.yaml                 (do ros2 bag)
    resumo_online.json            as estatísticas das deteções (o que a janela do resumo mostra)
```
* **Tópicos do bag:**

  | Tópico | Conteúdo |
  |---|---|
  | `/camara<N>/image_raw` | o vídeo original de cada câmara, como na aquisição (secção 1): cerca de **1 GB por minuto por câmara** |
  | `/camara<N>/processamento/dados_deteccoes` | uma mensagem JSON por imagem processada (abaixo) |
  | `/processamento/estado_camaras` | que câmaras estavam a ser processadas, a cada mudança (tecla D) |

  Confirme que há espaço livre no disco: com as duas câmaras, cerca de 2 GB por minuto, como na aquisição.
* **Cada mensagem de deteções** é uma string JSON com os mesmos campos do registo offline (secção 7.2):
  ```json
  {
    "objetos": [
      {"classe": "car", "confianca": 0.87, "caixa": [120, 210, 290, 340]}
    ],
    "linhas": [
      [[180.5, 479.0], [215.2, 420.0]]
    ],
    "estrada_percentagem": 0.34,
    "stamp_ns": 1795227005801337824,
    "camara": "5",
    "corrigida": true
  }
  ```
  * `stamp_ns` é o instante da imagem (o `header.stamp` posto pelo driver da câmara): **é o que liga cada deteção à imagem gravada no bag** (a mensagem de `/camara<N>/image_raw` com o mesmo `header.stamp`).
  * `corrigida` diz se as coordenadas (caixas e linhas) são as da imagem corrigida da distorção (com a calibração da secção 3) ou as da original.
  * Não se guardam os contornos da área circulável (só a percentagem), porque no online não se refaz o vídeo.
* **Relacionar as deteções com a imagem, mais tarde, no offline:** o bag tem o vídeo original, por isso pode ser processado com o `processar_bag.py` como um bag da aquisição (`python processar_bag.py data_realtime_inter/realtime_<data_hora>/realtime_<data_hora>_0.mcap`, secção 2), ou lido diretamente (cada deteção com a imagem do mesmo `stamp_ns`).
* As classes detetadas pelo YOLO são as mesmas do offline: `CLASSES_ESTRADA`, no `objetos.py` (vale para os dois). Para detetar as 80 classes do COCO, altere essa linha para `CLASSES_ESTRADA = None`. Como a pasta do projeto está montada em `/workspace`, **não precisa de reconstruir a imagem Docker** quando edita ficheiros `.py`.

### Testar o build e os modelos sem câmaras ligadas (Opcional)
Se quiser confirmar que o Docker constrói bem e que os três modelos ONNX correm em GPU ou CPU **sem ter as câmaras ligadas**:

1. **Construir apenas a imagem:**
   ```bash
   # Para testar com GPU NVIDIA (CUDA 13+):
   ONNX_PKG="onnxruntime-gpu[cuda,cudnn]" docker compose -f compose_realtime_inter.yaml build processador

   # Para testar com GPU NVIDIA (CUDA 12.x):
   ONNX_PKG="onnxruntime-gpu[cuda,cudnn]<1.27" docker compose -f compose_realtime_inter.yaml build processador

   # Para testar em CPU:
   docker compose -f compose_realtime_inter.yaml build processador
   ```

2. **Testar os três modelos numa imagem preta dentro do contentor:**
   ```bash
   # Teste em GPU (se construiu com onnxruntime-gpu e tem o toolkit NVIDIA ativo):
   docker compose -f compose_realtime_inter.yaml -f compose_realtime_gpu.yaml run --rm --no-deps processador python3 -c "
   import numpy as np, rclpy, objetos, linhas, estrada, visual
   img = np.zeros((480, 640, 3), dtype=np.uint8)
   for nome, det in [('Objetos', objetos.DetetorObjetos()), ('Linhas', linhas.DetetorLinhas()), ('Estrada', estrada.DetetorEstrada())]:
       det.detetar(img)
       print(f'{nome}: {det.dispositivo}')
   "

   # Teste em CPU (retira apenas o "-f compose_realtime_gpu.yaml"):
   docker compose -f compose_realtime_inter.yaml run --rm --no-deps processador python3 -c "
   import numpy as np, rclpy, objetos, linhas, estrada, visual
   img = np.zeros((480, 640, 3), dtype=np.uint8)
   for nome, det in [('Objetos', objetos.DetetorObjetos()), ('Linhas', linhas.DetetorLinhas()), ('Estrada', estrada.DetetorEstrada())]:
       det.detetar(img)
       print(f'{nome}: {det.dispositivo}')
   "
   ```
   Deve imprimir `Objetos`, `Linhas` e `Estrada` com `GPU (CUDA)` (ou `CPU`), sem erros. Os avisos `The "CAMARAS_ATIVAS" variable is not set` e `Found orphan containers (extract-rviz-1)` são normais quando se corre o `docker compose` diretamente fora do `processar_inter.sh`.

3. **Testar a janela do resumo** (depois de uma sessão): `docker compose -f compose_realtime_inter.yaml run --rm --no-deps processador python3 resumo_online.py data_realtime_inter/realtime_<data_hora>` (precisa do `xhost +local:root`).

### Problemas comuns (Secção 9)
* **`cannot open display` ou erro de janela gráfica:** esqueceu-se de executar `xhost +local:root` antes de lançar o script (Passo 2), ou pode adicionar `-i` ao comando se não precisar de abrir a interface gráfica.
* **`could not select device driver "nvidia"`:** tentou usar `compose_realtime_gpu.yaml` sem o *NVIDIA Container Toolkit* instalado/configurado (ver Passo 0). Se usar sempre `bash processar_inter.sh`, o script deteta isto sozinho e arranca em CPU sem dar erro.
* **Os modelos arrancam em `CPU` num PC com GPU:** confirme se correu `nvidia-smi` e `docker info | grep -i nvidia` (Passo 0) e confirme que não passou `USAR_GPU=0`.
* **Erro `FileNotFoundError` ao carregar os modelos:** confirme que a pasta `modelos/` existe na raiz do repositório com `objetos.onnx`, `linhas.onnx`, `linhas.json` e `estrada.onnx`.
* **Aviso `config/camera_astra_<N>.yaml não existe`:** falta a calibração dessa câmara (secção 3); as deteções dessa câmara são feitas na imagem original (`"corrigida": false`).
* **`Não foi gravado o resumo` no fim:** o processamento não chegou a arrancar (veja os erros do `processador` mais acima no terminal).
* **`alternar_camara.sh` diz que o processamento não está a correr:** corra-o na pasta do projeto, noutro terminal, enquanto o `processar_inter.sh` está a decorrer.

---

## Créditos
* CLRNet: Zheng et al., *CLRNet: Cross Layer Refinement Network for Lane Detection*, CVPR 2022.
* Implementação e pesos: [UnLanedet](https://github.com/zkyntu/UnLanedet) (licença Apache 2.0).
* Conjunto de dados de treino das linhas: CULane.
* Deteção da estrada: [YOLOP](https://github.com/hustvl/YOLOP), Wu et al., *YOLOP: You Only Look Once for Panoptic Driving Perception* (licença MIT), treinado no BDD100K.
* Mapa da janela de resultados: [OpenStreetMap](https://www.openstreetmap.org/copyright) (© contribuidores do OpenStreetMap), com o [TkinterMapView](https://github.com/TomSchimansky/TkinterMapView); janela com o [CustomTkinter](https://github.com/TomSchimansky/CustomTkinter).
* Deteção de objetos: [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) (licença AGPL-3.0), pré-treinado no COCO e utilizado em formato ONNX.
* Verifique as condições de utilização do CULane, do BDD100K, do UnLanedet, do YOLOP e do Ultralytics antes de usar os modelos fora do contexto académico.