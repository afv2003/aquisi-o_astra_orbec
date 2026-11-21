# -*- coding: utf-8 -*-
"""
preparar_ufld.py - Descarrega e verifica o modelo UFLDv2 (ONNX) para o comparar_linhas.py.

O ONNX do UFLDv2 ResNet34 CULane (convertido pelo PINTO0309, o indicado no README oficial do UFLDv2)
tem várias centenas de MB: não cabe no git (limite de 100 MB por ficheiro do GitHub). Por isso, cada
PC prepara-o uma vez com este script, que o guarda em modelos/ufld.onnx (no .gitignore).

  python preparar_ufld.py                              # descarrega e verifica
  python preparar_ufld.py --de resources.tar.gz        # usa o arquivo já descarregado
  python preparar_ufld.py --de ufldv2_culane_res34_320x1600.onnx   # ou o .onnx já extraído
  python preparar_ufld.py --verificar                  # só verifica o modelos/ufld.onnx

O arquivo do PINTO tem o modelo em vários formatos e tamanhos; o script lê-o à medida que o
descarrega e guarda só o ficheiro ufldv2_culane_res34_320x1600.onnx (pára logo que o encontra).
Verificação: entrada (1, 3, 320, 1600) e as quatro saídas do UFLDv2 (loc_row, loc_col, exist_row,
exist_col) com os tamanhos do CULane; depois, um teste numa imagem e o tempo por imagem.
"""
import argparse
import hashlib
import shutil
import sys
import tarfile
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ufld

NOME_ONNX = "ufldv2_culane_res34_320x1600.onnx"
DESTINO = ufld.PASTA_MODELOS / "ufld.onnx"
URL = ("https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/"
       "324_Ultra-Fast-Lane-Detection-v2/resources.tar.gz")
FORMAS = {"loc_row": (1, 200, 72, 4), "loc_col": (1, 100, 81, 4),
          "exist_row": (1, 2, 72, 4), "exist_col": (1, 2, 81, 4)}


class _Progresso:
    """Mostra os MB lidos de um ficheiro (ou da internet) enquanto o tar é percorrido."""

    def __init__(self, f, total=None):
        self.f, self.total, self.lidos, self.t = f, total, 0, 0.0

    def read(self, n=-1):
        dados = self.f.read(n)
        self.lidos += len(dados)
        if time.time() - self.t > 1:
            self.t = time.time()
            tot = f" de {self.total / 1e6:.0f}" if self.total else ""
            print(f"\r  {self.lidos / 1e6:.0f}{tot} MB lidos", end="", flush=True)
        return dados


def extrair_do_tar(fonte, total=None):
    """Percorre o tar.gz (ficheiro ou descarga) e grava o NOME_ONNX em DESTINO; True se o encontrou."""
    with tarfile.open(fileobj=_Progresso(fonte, total), mode="r|gz") as tar:
        for membro in tar:
            if membro.isfile() and Path(membro.name).name == NOME_ONNX:
                print(f"\n  encontrado: {membro.name} ({membro.size / 1e6:.0f} MB)")
                DESTINO.parent.mkdir(exist_ok=True)
                temporario = DESTINO.with_suffix(".parcial")
                with tar.extractfile(membro) as origem, open(temporario, "wb") as destino:
                    shutil.copyfileobj(origem, destino, 1 << 20)
                temporario.replace(DESTINO)
                return True
    print()
    return False


def verificar():
    """Confirma que o modelos/ufld.onnx é o UFLDv2 do CULane e corre um teste."""
    import numpy as np
    import onnxruntime as ort
    if not DESTINO.exists():
        print(f"Erro: {DESTINO} não existe. Corra: python preparar_ufld.py")
        return False
    print(f"A verificar {DESTINO} ({DESTINO.stat().st_size / 1e6:.0f} MB)...")
    try:
        s = ort.InferenceSession(str(DESTINO), providers=["CPUExecutionProvider"])
    except Exception as e:
        print(f"  ERRO: o ficheiro não é um modelo ONNX válido ({type(e).__name__}). A descarga pode ter "
              "ficado incompleta: apague o modelos/ufld.onnx e corra de novo o preparar_ufld.py.")
        return False
    ok = True
    e = s.get_inputs()[0]
    print(f"  entrada: {e.name} {e.shape}")
    if list(e.shape) != [1, 3, 320, 1600]:
        print("  ERRO: a entrada devia ser [1, 3, 320, 1600] (UFLDv2 CULane).")
        ok = False
    saidas = {o.name: o.shape for o in s.get_outputs()}
    for nome, forma in FORMAS.items():
        real = saidas.get(nome)
        estado = "ok" if real is not None and tuple(real) == forma else "ERRO"
        print(f"  saída {nome}: {real} (esperado {list(forma)}) {estado}")
        ok &= estado == "ok"
    if not ok:
        return False
    det = ufld.DetetorUFLD(DESTINO)
    img = np.full((480, 640, 3), 90, dtype=np.uint8)
    det.detetar(img)
    t0 = time.perf_counter()
    for _ in range(3):
        det.detetar(img)
    ms = (time.perf_counter() - t0) / 3 * 1000
    print(f"  teste numa imagem: ok ({det.dispositivo}, {ms:.0f} ms por imagem)")
    h = hashlib.sha256()
    with open(DESTINO, "rb") as f:
        for bloco in iter(lambda: f.read(1 << 20), b""):
            h.update(bloco)
    print(f"  sha256: {h.hexdigest()}")
    print("MODELO UFLD PRONTO (modelos/ufld.onnx). Pode usar: python comparar_linhas.py <bag ou vídeo>")
    return True


def main():
    ap = argparse.ArgumentParser(description="Descarrega e verifica o UFLDv2 (ONNX) para o comparar_linhas.py.")
    ap.add_argument("--de", type=str, default=None,
                    help=f"Usar um ficheiro já descarregado: o resources.tar.gz do PINTO ou o {NOME_ONNX}")
    ap.add_argument("--verificar", action="store_true", help="Só verificar o modelos/ufld.onnx")
    args = ap.parse_args()

    try:                                           # antes de descarregar: sem o onnxruntime não se verifica
        import onnxruntime  # noqa: F401
    except ImportError:
        print("Erro: falta o onnxruntime. Ative o venv do projeto (README, 3.8) e corra de novo:\n"
              "  venv\\Scripts\\activate          (Windows)\n"
              "  source venv/bin/activate       (Linux / macOS)")
        sys.exit(1)

    if not args.verificar:
        if args.de:
            fonte = Path(args.de)
            if not fonte.exists():
                print(f"Erro: '{fonte}' não existe.")
                sys.exit(1)
            if fonte.suffix == ".onnx":
                DESTINO.parent.mkdir(exist_ok=True)
                shutil.copyfile(fonte, DESTINO)
                print(f"Copiado para {DESTINO}.")
            else:
                print(f"A ler {fonte} à procura do {NOME_ONNX}...")
                with open(fonte, "rb") as f:
                    if not extrair_do_tar(f, fonte.stat().st_size):
                        print(f"Erro: o {NOME_ONNX} não está nesse arquivo.")
                        sys.exit(1)
        else:
            print(f"A descarregar do PINTO_model_zoo (só é guardado o {NOME_ONNX}):\n  {URL}")
            try:
                pedido = urllib.request.Request(URL, headers={"User-Agent": "aquisicao-astra"})
                with urllib.request.urlopen(pedido, timeout=60) as r:
                    total = int(r.headers.get("Content-Length") or 0) or None
                    if not extrair_do_tar(r, total):
                        print(f"Erro: o {NOME_ONNX} não está no arquivo descarregado.")
                        sys.exit(1)
            except OSError as e:
                print(f"\nErro a descarregar ({e}).")
                print("Alternativa: descarregue o arquivo noutro programa (ou com: curl -L -o resources.tar.gz "
                      f"{URL}) e corra: python preparar_ufld.py --de resources.tar.gz")
                sys.exit(1)
    sys.exit(0 if verificar() else 1)


if __name__ == "__main__":
    main()
