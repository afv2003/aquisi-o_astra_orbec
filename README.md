# Aquisição e Processamento de Imagem em ROS 2 (Jazzy)

Sistema em Docker para aquisição de imagem com câmara USB em ROS 2 Jazzy, visualização em tempo real no RViz2 e gravação automática em formato ROS bag (`.mcap`), acompanhado de scripts em Python/OpenCV para processamento offline.

## Pré-requisitos
* Docker e Docker Compose instalados
* Python 3 (com suporte a `venv`)

---

## 1. Aquisição de Dados (No Veículo)

O contentor Docker instala e executa o driver `usb_cam` (configurado para `/dev/video2`), abre o `rviz2` e inicia automaticamente a gravação do tópico `/image_raw` na pasta `./data`.

### Passo 1: Construir a imagem do Docker (Apenas na 1.ª vez)
Antes de utilizar o sistema pela primeira vez, é necessário descarregar e construir a imagem Docker com o ROS 2 e os pacotes da câmara. Abra o terminal na pasta do projeto e execute:
```bash
docker compose build
```

### Passo 2: Ligar e verificar a câmara USB
1. Ligue a câmara USB ao computador **antes** de iniciar o contentor.
2. > **Nota:** "Em princípio, a câmara externa deverá ser atribuída ao caminho `/dev/video2`, uma vez que o `/dev/video0` e o `/dev/video1` costumam estar reservados para a webcam integrada do portátil."
3. **Como confirmar ou alterar caso não seja `/dev/video2`:**
   * Para ver quais as câmaras detetadas pelo computador, corra no terminal:
     ```bash
     ls /dev/video*
     ```
   * Se a câmara externa tiver outro número (por exemplo, `/dev/video4`), abra o ficheiro `compose.yaml` num editor de texto (ou com `nano compose.yaml`), procure a linha do serviço `driver_camara`:
     ```yaml
     command: ros2 run usb_cam usb_cam_node_exe --ros-args -p frame_id:=base_link -p video_device:=/dev/video2
     ```
     e altere `/dev/video2` para o número correto correspondente à sua câmara. Guarde o ficheiro antes de avançar.

### Passo 3: Autorizar o uso do ecrã (Apenas uma vez depois de ligar o PC)
Antes de iniciar o Docker, dê permissão para que o RViz consiga abrir a janela gráfica. Só precisa de executar este comando uma vez sempre que liga o computador:
```bash
xhost +local:root
```

### Passo 4: Iniciar a aquisição e gravação
Com a câmara ligada, inicie o sistema indicando **qual das câmaras** (5 ou 8) está ligada:
```bash
CAMARA=5 docker compose up
```
* O `CAMARA` é obrigatório: sem ele, o Docker não arranca e mostra `defina CAMARA (5 ou 8)`. Assim cada gravação fica associada à câmara que a fez.
* A câmara irá ligar, a janela do **RViz2** abrirá automaticamente para visualização e a gravação do ficheiro `.mcap` começará de imediato dentro da pasta `data/`.

### Passo 5: Terminar e guardar a gravação
1. Quando quiser parar de gravar, vá ao terminal onde correu o comando anterior e prima **`Ctrl + C`** (apenas uma vez).
2. Aguarde alguns segundos até que o Docker feche todos os processos em segurança.
3. Os dados gravados ficarão automaticamente disponíveis no seu computador dentro de uma subpasta com a data e hora da recolha (exemplo: `data/viagem_camara5_AAAAMMDD_HHMMSS/`). O nome inclui a câmara usada.

---

## 2. Processamento de Imagem / Extração (Offline)

O tratamento das imagens com OpenCV ou a extração para vídeo é feito diretamente no computador através de um ambiente virtual Python (`venv`), sem necessidade de ligar a câmara ou o Docker.

### Passo 1: Criar o ambiente virtual `venv` (Apenas na 1.ª vez)
Quando descarregar o repositório pela primeira vez, abra o terminal na pasta do projeto e execute os comandos abaixo para criar o ambiente isolado e instalar as bibliotecas necessárias (`opencv-python`, `numpy` e `mcap-ros2-support`):
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### Passo 2: Como executar o programa Python (Utilização normal)
Sempre que abrir um terminal novo para processar um ficheiro `.mcap` já gravado, siga esta sequência:

1. **Ativar o ambiente virtual:**
   ```bash
   source venv/bin/activate
   ```
   *(Saberá que está ativo porque aparecerá `(venv)` no início da linha do terminal).*

2. **Verificar o nome do ficheiro `.mcap` gravado (Opcional):**
   Se não souber o nome exato da pasta que o Docker criou, liste os ficheiros `.mcap` existentes com:
   ```bash
   find data/ -name "*.mcap"
   ```

3. **Executar o script Python:**
   Corra o programa `processar_bag.py` indicando à frente o caminho completo para o ficheiro `.mcap` que pretende processar:
   ```bash
   python processar_bag.py data/<nome_da_pasta>/<nome_do_ficheiro>.mcap
   ```
   *(Exemplo prático: `python processar_bag.py data/viagem_camara5_20261106_101500/viagem_camara5_20261106_101500_0.mcap`)*
   * Use `-v` para ver só o vídeo, sem YOLO. A imagem é corrigida com a calibração da câmara que gravou o bag (ver secção 3).

4. **Sair do ambiente virtual (Quando terminar o trabalho):**
   ```bash
   deactivate
   ```

---

## 3. Calibração da câmara (correção da distorção da lente)

A lente da Astra curva ligeiramente as linhas retas, sobretudo nos cantos da imagem. A calibração mede essa distorção uma única vez e o `processar_bag.py` corrige-a em cada frame, antes do YOLO.

### Passo 1: Gerar a calibração (Windows, uma vez por câmara)
Há duas câmaras Astra, identificadas como **5** e **8**. Cada uma tem a sua calibração, porque duas unidades do mesmo modelo nunca são exatamente iguais.

Num PC com Windows (sem ROS), com a câmara ligada e um xadrez impresso:
```bash
pip install opencv-python numpy
python calibrar_astra.py --camara 5 --cam 1 --colunas 9 --linhas 7 --quadrado 25 --auto
```
* `--camara` é a designação da câmara física (**5** ou **8**), obrigatória.
* `--colunas`/`--linhas` são os **cantos internos** do xadrez (10x7 quadrados → 9x6 cantos).
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

### Passo 2: Copiar a calibração para este projeto
Copie o ficheiro gerado de cada câmara para a pasta `config/` deste repositório:
```
config/camera_astra_5.yaml
config/camera_astra_8.yaml
```

### Passo 3: Usar
* **Processamento offline:** o `processar_bag.py` lê a câmara do nome do bag (`viagem_camara5_...`) e usa `config/camera_astra_<N>.yaml`:
  ```bash
  python processar_bag.py data/<pasta>/<ficheiro>.mcap                  # câmara lida do nome do bag
  python processar_bag.py data/<pasta_antiga>/<ficheiro>.mcap --camara 5   # bags antigos, sem camara<N> no nome
  python processar_bag.py data/<pasta>/<ficheiro>.mcap --calib outro.yaml
  python processar_bag.py data/<pasta>/<ficheiro>.mcap --sem-correcao
  ```
  * Se indicar `--camara` diferente da do nome do bag, o script para com erro, para não usar a calibração errada.
  * Num bag sem `camara<N>` no nome (gravado antes desta alteração), é obrigatório indicar `--camara`.
  Durante a reprodução, a tecla **C** liga/desliga a correção para comparar.
* Corra o script a partir da pasta do repositório; noutra pasta, indique o ficheiro com `--calib`.
* Não renomeie as pastas dos bags: a designação `camara<N>` no nome é o que liga cada gravação à sua calibração.

> **Importante:** a calibração só é válida para a resolução usada (640x480, a resolução por omissão do `usb_cam`). Se mudar a resolução, a focagem ou a lente, repita a calibração.