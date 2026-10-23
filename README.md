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

### Passo 3: Iniciar a aquisição e gravação
Com a câmara ligada, inicie o sistema executando no terminal:
```bash
docker compose up
```
* A câmara irá ligar, a janela do **RViz2** abrirá automaticamente para visualização e a gravação do ficheiro `.mcap` começará de imediato dentro da pasta `data/`.

### Passo 4: Terminar e guardar a gravação
1. Quando quiser parar de gravar, vá ao terminal onde correu o comando anterior e prima **`Ctrl + C`** (apenas uma vez).
2. Aguarde alguns segundos até que o Docker feche todos os processos em segurança.
3. Os dados gravados ficarão automaticamente disponíveis no seu computador dentro de uma subpasta com a data e hora da recolha (exemplo: `data/viagem_AAAAMMDD_HHMMSS/`).

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
   Corra o programa `extrair_video.py` indicando à frente o caminho completo para o ficheiro `.mcap` que pretende processar:
   ```bash
   python extrair_video.py data/<nome_da_pasta>/<nome_do_ficheiro>.mcap
   ```
   *(Exemplo prático: `python extrair_video.py data/viagem_20261023_153000/viagem_20261023_153000_0.mcap`)*

4. **Sair do ambiente virtual (Quando terminar o trabalho):**
   ```bash
   deactivate
   ```
