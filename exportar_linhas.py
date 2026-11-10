#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
exportar_linhas.py - Exporta o CLRNet / CLRerNet do UnLanedet para ONNX e VERIFICA a exportação.

Corre UMA vez, no "ambiente de exportação" (venv próprio, ver README secção 4).
Gera modelos/linhas.onnx e modelos/linhas.json, que vão para o repositório; a
partir daí o processar_bag.py usa-os em qualquer PC, só com o onnxruntime.

Não precisa de GPU nem de compilar nada: as partes do UnLanedet escritas em CUDA
(NMS e DCN) não fazem parte da rede e são substituídas por módulos vazios.

Uso (na pasta do repositório, com o venv de exportação ativo):
  python exportar_linhas.py --unlanedet ../UnLanedet
  python exportar_linhas.py --unlanedet ../UnLanedet --modelo clrernet_r34
  python exportar_linhas.py --unlanedet ../UnLanedet --imagem frame_de_teste.png

Verificações feitas no fim:
  1. o ficheiro ONNX é válido (onnx.checker);
  2. a saída do ONNX (onnxruntime) é igual à do PyTorch;
  3. as linhas descodificadas a partir das duas saídas são iguais;
  4. se o NMS compilado do UnLanedet existir, as linhas são iguais às do UnLanedet.
"""

import argparse
import datetime
import importlib
import inspect
import json
import os
import sys
import time
import types
import urllib.request
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent
URL_PESOS = "https://github.com/zkyntu/UnLanedet/releases/download/Weights/"

# Nota: na tabela do Model Zoo do UnLanedet os links do CLRNet ResNet34 e ResNet50 estão
# trocados. Os nomes abaixo foram confirmados pelos parâmetros dentro de cada ficheiro.
MODELOS = {
    "clrnet_r34": ("config/clrnet/resnet34_culane.py", "clrnet_model_best_culane.pth",
                   "CLRNet ResNet34, CULane (F1 78.99 no UnLanedet)"),
    "clrnet_r50": ("config/clrnet/resnet50_culane.py", "clrnet_r50_culane_model_best.pth",
                   "CLRNet ResNet50, CULane (F1 79.30 no UnLanedet)"),
    "clrernet_r34": ("config/clrernet/resnet34_culane.py", "clrernet_model_best_culane.pth",
                     "CLRerNet ResNet34, CULane (F1 79.20 no UnLanedet)"),
}

OPS_COMPILADAS = ["unlanedet.layers.ops.nms_impl", "unlanedet.layers.ops.nms_ad_impl",
                  "unlanedet.layers.ops.nms_sr_impl", "unlanedet.layers.ops.dcn.deform_conv_ext"]


def nms_compilado_existe(pasta_unlanedet):
    ops = pasta_unlanedet / "unlanedet" / "layers" / "ops"
    return any(ops.glob("nms_impl*.so")) or any(ops.glob("nms_impl*.pyd"))


def substituir_ops_compiladas():
    """Módulos vazios no lugar das extensões CUDA (não são usadas pela rede)."""
    for nome in OPS_COMPILADAS:
        m = types.ModuleType(nome)

        def falta(*a, _n=nome, **k):
            raise RuntimeError(f"{_n} não está compilado (não é preciso para exportar)")

        def atributo(attr, f=falta):
            if attr.startswith("__"):      # __file__, __spec__, ...: comportamento normal de um módulo
                raise AttributeError(attr)
            return f                        # qualquer função do módulo compilado
        m.__getattr__ = atributo
        sys.modules[nome] = m


def descarregar(url, destino):
    print(f"A descarregar {url}\n  -> {destino}")
    destino.parent.mkdir(parents=True, exist_ok=True)
    tmp = destino.with_suffix(".parcial")
    urllib.request.urlretrieve(url, tmp)
    tmp.replace(destino)


def main():
    ap = argparse.ArgumentParser(description="Exportar CLRNet/CLRerNet (UnLanedet) para ONNX.")
    ap.add_argument("--unlanedet", required=True, help="pasta do repositório UnLanedet (git clone)")
    ap.add_argument("--modelo", choices=sorted(MODELOS), default="clrnet_r34")
    ap.add_argument("--pesos", help="ficheiro .pth (por omissão é descarregado do UnLanedet)")
    ap.add_argument("--imagem", help="imagem de teste (ex.: um frame de um bag) para ver o resultado")
    ap.add_argument("--saida", default=str(REPO / "modelos"), help="pasta onde gravar linhas.onnx/.json")
    ap.add_argument("--opset", type=int, default=17)
    args = ap.parse_args()

    pasta_ul = Path(args.unlanedet).resolve()
    if not (pasta_ul / "unlanedet").is_dir():
        sys.exit(f"Erro: '{pasta_ul}' não parece ser o repositório UnLanedet (falta a pasta unlanedet/).")
    ficheiro_cfg, ficheiro_pesos, descricao = MODELOS[args.modelo]
    pesos = Path(args.pesos).resolve() if args.pesos else (REPO / "pesos" / ficheiro_pesos)
    if not pesos.exists():
        descarregar(URL_PESOS + ficheiro_pesos, pesos)
    saida = Path(args.saida).resolve()
    saida.mkdir(parents=True, exist_ok=True)
    imagem = Path(args.imagem).resolve() if args.imagem else None

    # --- importar o UnLanedet a partir da sua pasta ---
    sys.path.insert(0, str(REPO))          # linhas.py
    sys.path.insert(0, str(pasta_ul))      # unlanedet + config
    os.chdir(pasta_ul)
    import cv2
    import torch
    import onnx
    import onnxruntime as ort

    # só depois do torch: módulos vazios no lugar das extensões CUDA do UnLanedet
    tem_nms_real = nms_compilado_existe(pasta_ul)
    if not tem_nms_real:
        substituir_ops_compiladas()
    from unlanedet.config import LazyConfig, instantiate
    import linhas

    print(f"\nModelo: {descricao}")
    print(f"PyTorch {torch.__version__} | onnx {onnx.__version__} | onnxruntime {ort.__version__}")
    print(f"NMS compilado do UnLanedet: {'sim' if tem_nms_real else 'não (substituído; não é preciso)'}")

    cfg = LazyConfig.load(ficheiro_cfg)
    cfg.model.backbone.pretrained = False   # os pesos vêm do .pth, não da internet
    pc = cfg.model.head.cfg
    cfg_linhas = {
        "img_w": int(pc.img_w), "img_h": int(pc.img_h), "n_offsets": int(pc.num_points),
        "conf_threshold": float(pc.test_parameters.conf_threshold),
        "nms_thres": float(pc.test_parameters.nms_thres), "max_lanes": int(pc.max_lanes),
        "corte_topo": float(pc.cut_height) / float(pc.ori_img_h),
    }

    modelo = instantiate(cfg.model)
    ckpt = torch.load(str(pesos), map_location="cpu", weights_only=False)
    estado = ckpt.get("model", ckpt)
    modelo.load_state_dict(estado, strict=True)   # falha se o .pth não for deste modelo
    modelo.eval()
    print(f"Pesos carregados: {pesos.name} ({sum(p.numel() for p in modelo.parameters()) / 1e6:.1f} M parâmetros)")

    class ParaExportar(torch.nn.Module):
        def __init__(self, m):
            super().__init__()
            self.m = m

        def forward(self, img):
            return self.m({"img": img})   # (1, 192, 78): [2 logits, start_y, start_x, theta, comprimento, 72 x]

    rede = ParaExportar(modelo).eval()

    # entrada de teste: a imagem indicada, ou uma imagem sintética
    if imagem is not None:
        img_bgr = cv2.imread(str(imagem))
        if img_bgr is None:
            sys.exit(f"Erro: não consegui ler a imagem {imagem}")
    else:
        rng = np.random.default_rng(0)
        img_bgr = (rng.random((590, 1640, 3)) * 255).astype(np.uint8)
    x_np, corte_px = linhas.preprocessar(img_bgr, cfg_linhas)
    x = torch.from_numpy(x_np)

    with torch.no_grad():
        saida_torch = rede(x).numpy()

    # --- exportação ---
    caminho_onnx = saida / "linhas.onnx"
    opcoes = dict(opset_version=args.opset, input_names=["imagem"], output_names=["propostas"],
                  do_constant_folding=True)
    if "dynamo" in inspect.signature(torch.onnx.export).parameters:
        opcoes["dynamo"] = False   # exportador clássico (TorchScript), o testado com este modelo
    print(f"\nA exportar para {caminho_onnx} (opset {args.opset}) ...")
    # torch.flip não é exportável em todas as versões do PyTorch: durante a exportação usa-se
    # uma versão equivalente (index_select com índices invertidos), que dá exatamente o mesmo resultado.
    flip_original = torch.flip

    def flip_exportavel(t, dims):
        for d in dims:
            t = t.index_select(d, torch.arange(t.shape[d] - 1, -1, -1, device=t.device))
        return t
    torch.flip = flip_exportavel
    try:
        torch.onnx.export(rede, (x,), str(caminho_onnx), **opcoes)
    finally:
        torch.flip = flip_original
    onnx.checker.check_model(str(caminho_onnx))
    tamanho_mb = caminho_onnx.stat().st_size / 1e6
    print(f"[1] ONNX válido ({tamanho_mb:.0f} MB)")
    if tamanho_mb > 100:
        print("    AVISO: mais de 100 MB; o GitHub não aceita ficheiros deste tamanho (use uma Release).")

    # --- verificação 2: saída ONNX == saída PyTorch ---
    sessao = ort.InferenceSession(str(caminho_onnx), providers=["CPUExecutionProvider"])
    saida_onnx = sessao.run(None, {"imagem": x_np})[0]
    dif = float(np.abs(saida_onnx - saida_torch).max())
    ok2 = dif < 1e-3
    print(f"[2] Diferença máxima PyTorch vs ONNX: {dif:.2e}  -> {'OK' if ok2 else 'DIFERENTE!'}")

    # --- verificação 3: linhas descodificadas iguais ---
    h, w = img_bgr.shape[:2]
    l_torch = linhas.descodificar(saida_torch[0], cfg_linhas, w, h, corte_px)
    l_onnx = linhas.descodificar(saida_onnx[0], cfg_linhas, w, h, corte_px)
    ok3 = len(l_torch) == len(l_onnx) and all(
        a.shape == b.shape and np.abs(a - b).max() < 1.0 for a, b in zip(l_torch, l_onnx))
    print(f"[3] Linhas a partir do PyTorch: {len(l_torch)} | a partir do ONNX: {len(l_onnx)}"
          f"  -> {'OK' if ok3 else 'DIFERENTE!'}")

    # --- verificação 4: igual ao pós-processamento original do UnLanedet ---
    # (o NMS compilado do UnLanedet só corre em GPU)
    ok4 = None
    if tem_nms_real and torch.cuda.is_available():
        W, H = int(pc.ori_img_w), int(pc.ori_img_h)
        ref = cv2.resize(img_bgr, (W, H))
        xr, cr = linhas.preprocessar(ref, cfg_linhas)
        rede.cuda()
        with torch.no_grad():
            sr = rede(torch.from_numpy(xr).cuda())
            lanes_ul = modelo.get_lanes(sr)[0]
        rede.cpu()
        ul = []
        for ln in lanes_ul:
            pts = np.asarray(ln.points, dtype=np.float64) * np.array([W, H])
            pts = pts[(pts[:, 0] >= 0) & (pts[:, 0] < W)]
            if len(pts) > 1:
                ul.append(pts)
        nos = linhas.descodificar(sr.cpu().numpy()[0], cfg_linhas, W, H, cr)
        ok4 = len(ul) == len(nos) and all(
            a.shape == b.shape and np.abs(a - b).max() < 1.0 for a, b in zip(ul, nos))
        print(f"[4] Linhas UnLanedet original: {len(ul)} | linhas.py: {len(nos)}  -> {'OK' if ok4 else 'DIFERENTE!'}")
    else:
        print("[4] Comparação com o NMS original do UnLanedet: saltada "
              "(precisa do NMS compilado e de GPU; as verificações 2 e 3 chegam para validar o ONNX).")

    # --- velocidade em CPU ---
    t0 = time.time()
    for _ in range(5):
        sessao.run(None, {"imagem": x_np})
    fps = 5 / (time.time() - t0)
    print(f"Velocidade em CPU (onnxruntime): {fps:.1f} imagens/s")

    # --- configuração para o linhas.py ---
    cfg_linhas.update({
        "modelo": args.modelo, "descricao": descricao, "pesos": ficheiro_pesos,
        "origem": "https://github.com/zkyntu/UnLanedet", "opset": args.opset,
        "exportado_em": datetime.datetime.now().isoformat(timespec="seconds"),
    })
    (saida / "linhas.json").write_text(json.dumps(cfg_linhas, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Configuração gravada em {saida / 'linhas.json'}")

    if imagem is not None:
        vis = linhas.desenhar_linhas(img_bgr.copy(), l_onnx)
        destino = saida / "teste_linhas.png"
        cv2.imwrite(str(destino), vis)
        print(f"Resultado na imagem de teste: {destino}  ({len(l_onnx)} linhas)")

    tudo_ok = ok2 and ok3 and (ok4 is not False)
    print("\nEXPORTAÇÃO VERIFICADA COM SUCESSO" if tudo_ok else "\nATENÇÃO: alguma verificação falhou (ver acima)")
    sys.exit(0 if tudo_ok else 1)


if __name__ == "__main__":
    main()
