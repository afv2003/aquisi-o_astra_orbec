#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrar_astra.py
=================
Calibração intrínseca da câmara RGB Orbbec Astra em Windows (sem ROS), com xadrez,
deteção em tempo real, captura automática ("dinâmica") e revisão das fotos.

Fluxo
-----
1. Abre a câmara em tempo real e deteta o xadrez em cada frame.
2. Captura manual (ESPAÇO) ou automática (tecla A): só tira foto quando o xadrez
   está parado E numa pose suficientemente diferente das anteriores.
   Um mapa de cobertura (verde) mostra as zonas da imagem já cobertas e 4 barras
   (X, Y, Tam, Incl) mostram o progresso, como na ferramenta do ROS.
   R, C e Q só ficam disponíveis com as 4 barras cheias (ou 40 fotos).
3. R: desliga a captura automática e revê as fotos com o erro de cada uma;
   O remove os outliers; ENTER volta. (A volta a ligar a captura automática.)
4. C: calibra com as fotos que ficaram.
5. U: mostra a imagem já corrigida (undistort) para validar.
6. Q: grava e sai. Se os outliers não tiverem sido removidos (R -> O -> C),
   aparece um aviso; um segundo Q grava assim mesmo, outra tecla cancela.
   ESC sai sem gravar.

Saídas (pasta --saida/camara<N>_<data_hora>/)
-----------------------------------
  imagens/                     fotos capturadas (PNG, resolução total)
  camera_astra_<N>.yaml        calibração (formato ROS camera_info) -> config/ do repositório
  calibracao.json              mesmos valores + relatório (RMS, erro por foto, rejeitadas)

Requisitos
----------
  pip install opencv-python numpy

Exemplos
--------
  python calibrar_astra.py --listar
  python calibrar_astra.py --camara 5 --cam 1 --colunas 9 --linhas 7 --quadrado 25 --auto
  python calibrar_astra.py --camara 8 --cam 1 --colunas 9 --linhas 7 --quadrado 25 --auto

Critérios iguais aos da ferramenta camera_calibration do ROS
------------------------------------------------------------
* Só k1 e k2 são estimados (k3 = 0), como o '-k 2' do ROS.
* Xadrez com cantos a menos de 8 px da borda da imagem é ignorado.
* Uma foto nova só é aceite se diferir das anteriores em mais de 0.2 (X+Y+Tam+Incl).
* Fotos suficientes = 4 barras cheias (amplitudes 0.7, 0.7, 0.4, 0.5) ou 40 fotos.
* Na vista corrigida (U) mostra o erro de linearidade 'lin.' em px.
* A mais que o ROS: exige o xadrez parado e permite remover outliers (R -> O).

Notas importantes
-----------------
* --colunas/--linhas são CANTOS INTERNOS (um xadrez de 10x7 quadrados tem 9x6 cantos).
  Prefira um nº par x ímpar (ex. 9x6) para a orientação não ser ambígua.
* A calibração é válida para a RESOLUÇÃO usada. O usb_cam grava 640x480 por
  omissão, que é também o valor por omissão deste script.
"""

import argparse
import datetime
import json
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# --------------------------------------------------------------------------- #
# Constantes / teclas
# --------------------------------------------------------------------------- #
JANELA = "Calibracao Astra"
JANELA_REV = "Revisao das capturas"
MIN_IMAGENS = 8
GRELHA_COB = (8, 6)  # células do mapa de cobertura (x, y)

HAS_SB = hasattr(cv2, "findChessboardCornersSB")
CRIT_SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
CRIT_CALIB = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-9)

# códigos de waitKeyEx (Windows / Linux GTK)
K_ESQ = {2424832, 65361, ord("a"), ord("p")}
K_DIR = {2555904, 65363, ord("d"), ord("n"), ord(" ")}
K_DEL = {3014656, 65535, 8, ord("x")}
K_ENTER = {13, 10}
K_ESC = 27


def tecla(k):
    """Normaliza o código devolvido por waitKeyEx."""
    if k is None or k < 0:
        return -1
    if k in K_ESQ | K_DIR | K_DEL:
        return k
    return k & 0xFF


# --------------------------------------------------------------------------- #
# Fontes de imagem
# --------------------------------------------------------------------------- #
class FonteUVC:
    """RGB da Astra (UVC) - o mesmo caminho que o usb_cam usa em Linux."""

    def __init__(self, indice, largura, altura, fps, mjpg):
        backend = cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(indice, backend)
        if not self.cap.isOpened() and backend != cv2.CAP_ANY:
            self.cap = cv2.VideoCapture(indice, cv2.CAP_MSMF)
        if not self.cap.isOpened():
            raise RuntimeError(f"Não foi possível abrir a câmara {indice}. "
                               f"Experimente --listar ou outro --cam.")
        if mjpg:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, largura)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, altura)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # autofocus desligado se suportado (a Astra normalmente é foco fixo)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[UVC] câmara {indice} aberta em {w}x{h}")
        if (w, h) != (largura, altura):
            print(f"[AVISO] pediu {largura}x{altura} mas a câmara devolveu {w}x{h}.")

    def ler(self):
        return self.cap.read()

    def fechar(self):
        self.cap.release()


def listar_cameras(maximo=6):
    backend = cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_ANY
    print("A procurar câmaras UVC...")
    for i in range(maximo):
        cap = cv2.VideoCapture(i, backend)
        if cap.isOpened():
            ok, fr = cap.read()
            res = f"{fr.shape[1]}x{fr.shape[0]}" if ok else "sem frame"
            print(f"  índice {i}: aberta ({res})")
        cap.release()


# --------------------------------------------------------------------------- #
# Deteção do xadrez
# --------------------------------------------------------------------------- #
def janela_subpix(corners, padrao):
    c = corners.reshape(padrao[1], padrao[0], 2)
    d = np.linalg.norm(c[:, 1:] - c[:, :-1], axis=2).min()
    r = int(np.clip(d / 4, 3, 11))
    return (r, r)


def detetar_xadrez(gray, padrao, rapido=True):
    """Devolve (ok, cantos Nx1x2 float32)."""
    if HAS_SB:
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        if not rapido:
            flags |= cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCornersSB(gray, padrao, flags=flags)
        if ok:
            corners = corners.reshape(-1, 1, 2).astype(np.float32)
            if not rapido:  # refinamento sub-pixel final (mais exato que o SB sozinho)
                corners = cv2.cornerSubPix(gray, corners, janela_subpix(corners, padrao),
                                           (-1, -1), CRIT_SUBPIX)
            return True, corners
        if rapido:
            return False, None
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    if rapido:
        flags |= cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, padrao, flags=flags)
    if ok:
        corners = corners.reshape(-1, 1, 2).astype(np.float32)
        corners = cv2.cornerSubPix(gray, corners, janela_subpix(corners, padrao),
                                   (-1, -1), CRIT_SUBPIX)
    return ok, corners


# ---- Critérios da ferramenta camera_calibration do ROS ----
BORDA_ROS = 8                          # px: rejeitar xadrez com cantos junto à borda
INTERVALOS_ROS = [0.7, 0.7, 0.4, 0.5]  # X, Y, Size, Skew: amplitude p/ encher cada barra
MIN_FOTOS_ROS = 40                     # ou 40 fotos, mesmo sem as barras cheias


def perto_da_borda(corners, w, h, borda=BORDA_ROS):
    c = corners.reshape(-1, 2)
    return not np.all((c[:, 0] > borda) & (c[:, 0] < w - borda) &
                      (c[:, 1] > borda) & (c[:, 1] < h - borda))


def _angulo(a, b, c):
    ab, cb = a - b, c - b
    return float(np.arccos(np.clip(np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb)), -1, 1)))


def parametros_ros(corners, padrao, w, h):
    """[X, Y, Size, Skew] como no ROS: posição, tamanho e inclinação do xadrez (0..1)."""
    c = corners.reshape(-1, 2).astype(np.float64)
    cols = padrao[0]
    ul, ur, dr, dl = c[0], c[cols - 1], c[-1], c[-cols]
    a, b, cc = ur - ul, dr - ur, dl - dr
    p, q = b + cc, a + b
    area = abs(p[0] * q[1] - p[1] * q[0]) / 2.0
    borda = np.sqrt(area)
    px = min(1.0, max(0.0, (c[:, 0].mean() - borda / 2) / (w - borda)))
    py = min(1.0, max(0.0, (c[:, 1].mean() - borda / 2) / (h - borda)))
    tamanho = np.sqrt(area / (w * h))
    skew = min(1.0, 2.0 * abs(np.pi / 2 - _angulo(ul, ur, dr)))
    return np.array([px, py, tamanho, skew])


def distancia_ros(p1, p2):
    return float(np.sum(np.abs(np.asarray(p1) - np.asarray(p2))))


def progresso_ros(lista_params):
    """Barras X, Y, Size, Skew (0..1) e se já há fotos suficientes (critério do ROS)."""
    if not lista_params:
        return [0.0, 0.0, 0.0, 0.0], False
    m = np.array(lista_params)
    mn, mx = m.min(0), m.max(0)
    mn[2] = mn[3] = 0.0  # como no ROS: não premiar xadrez pequeno nem pouco inclinado
    prog = [min((hi - lo) / r, 1.0) for lo, hi, r in zip(mn, mx, INTERVALOS_ROS)]
    return prog, (len(lista_params) >= MIN_FOTOS_ROS or all(v >= 1.0 for v in prog))


def erro_linearidade(pontos, padrao):
    """'lin.' do ROS: RMS da distância (px) de cada canto à reta da sua linha do xadrez."""
    g = pontos.reshape(padrao[1], padrao[0], 2).astype(np.float64)
    erros = []
    for linha in g:
        a, b = linha[0], linha[-1]
        d = b - a
        n = np.linalg.norm(d)
        for p in linha[1:-1]:
            erros.append(abs(d[0] * (p[1] - a[1]) - d[1] * (p[0] - a[0])) / n)
    return float(np.sqrt(np.mean(np.square(erros)))) if erros else 0.0


def nitidez(gray, corners):
    x, y, w, h = cv2.boundingRect(corners.reshape(-1, 2).astype(np.float32))
    roi = gray[max(y, 0):y + h, max(x, 0):x + w]
    return cv2.Laplacian(roi, cv2.CV_64F).var() if roi.size else 0.0


# --------------------------------------------------------------------------- #
# Calibração
# --------------------------------------------------------------------------- #
def pontos_objeto(padrao, quadrado):
    objp = np.zeros((padrao[0] * padrao[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:padrao[0], 0:padrao[1]].T.reshape(-1, 2) * quadrado
    return objp


def calibrar(capturas, padrao, quadrado, tamanho, flags, alpha):
    validas = [c for c in capturas if not c["rejeitada"]]
    if len(validas) < MIN_IMAGENS:
        print(f"[!] São precisas pelo menos {MIN_IMAGENS} imagens válidas "
              f"(tem {len(validas)}).")
        return None
    objp = pontos_objeto(padrao, quadrado)
    obj = [objp] * len(validas)
    img = [c["corners"] for c in validas]
    print(f"\nA calibrar com {len(validas)} imagens ({tamanho[0]}x{tamanho[1]})...")
    rms, K, D, rvecs, tvecs, std_int, _std_ext, per_view = cv2.calibrateCameraExtended(
        obj, img, tamanho, None, None, flags=flags, criteria=CRIT_CALIB)

    # erro e reprojeção para TODAS (rejeitadas via solvePnP, para poder reavaliar)
    i_val = 0
    for c in capturas:
        if not c["rejeitada"]:
            rv, tv = rvecs[i_val], tvecs[i_val]
            c["erro"] = float(per_view[i_val][0])
            i_val += 1
        else:
            ok, rv, tv = cv2.solvePnP(objp, c["corners"], K, D)
            if not ok:
                c["erro"], c["reproj"] = float("nan"), None
                continue
        proj, _ = cv2.projectPoints(objp, rv, tv, K, D)
        c["reproj"] = proj
        if c["rejeitada"]:
            c["erro"] = float(np.sqrt(np.mean(
                np.sum((proj - c["corners"]) ** 2, axis=2))))

    newK, roi = cv2.getOptimalNewCameraMatrix(K, D, tamanho, alpha, tamanho)
    std = std_int.ravel()
    res = dict(rms=float(rms), K=K, D=D, newK=newK, roi=tuple(int(v) for v in roi),
               tamanho=tamanho, n=len(validas),
               std=dict(fx=float(std[0]), fy=float(std[1]),
                        cx=float(std[2]), cy=float(std[3])),
               alpha=alpha)
    imprimir_resultado(res)
    return res


def imprimir_resultado(res):
    K, D = res["K"], res["D"].ravel()
    s = res["std"]
    q = "BOA" if res["rms"] < 0.5 else ("ACEITÁVEL" if res["rms"] < 1.0 else "FRACA - rever fotos")
    print("-" * 60)
    print(f" RMS de reprojeção : {res['rms']:.4f} px  ({q})")
    print(f" fx = {K[0,0]:.3f} ± {s['fx']:.3f}   fy = {K[1,1]:.3f} ± {s['fy']:.3f}")
    print(f" cx = {K[0,2]:.3f} ± {s['cx']:.3f}   cy = {K[1,2]:.3f} ± {s['cy']:.3f}")
    print(f" D  = {np.array2string(D, precision=6, separator=', ')}")
    print("-" * 60)


# --------------------------------------------------------------------------- #
# Gravação dos resultados
# --------------------------------------------------------------------------- #
def _lista(a):
    return ", ".join(f"{v:.10g}" for v in np.asarray(a).ravel())


def gravar_resultados(res, capturas, args, pasta):
    pasta.mkdir(parents=True, exist_ok=True)
    K, D, newK = res["K"], res["D"].reshape(1, -1), res["newK"]
    w, h = res["tamanho"]
    cam = getattr(args, "camara", None)
    nome = f"astra_rgb_{cam}" if cam else "astra_rgb"
    fich_yaml = f"camera_astra_{cam}.yaml" if cam else "camera_astra.yaml"
    P = np.hstack([newK, np.zeros((3, 1))])

    # 1) ROS camera_info
    ros = (f"image_width: {w}\nimage_height: {h}\ncamera_name: {nome}\n"
           f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: [{_lista(K)}]\n"
           f"distortion_model: plumb_bob\n"
           f"distortion_coefficients:\n  rows: 1\n  cols: {D.size}\n  data: [{_lista(D)}]\n"
           f"rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
           f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: [{_lista(P)}]\n")
    (pasta / fich_yaml).write_text(ros, encoding="utf-8")

    # 2) JSON: os mesmos valores + relatório (RMS, erro por foto, rejeitadas)
    info = dict(
        data=datetime.datetime.now().isoformat(timespec="seconds"),
        camara=cam,
        camera=nome, image_width=w, image_height=h,
        padrao_cantos_internos=[args.colunas, args.linhas],
        tamanho_quadrado=args.quadrado, unidade=args.unidade,
        rms_px=res["rms"], n_imagens=res["n"], desvio_padrao=res["std"],
        k_coeficientes=getattr(args, "k_coeficientes", 2),
        camera_matrix=K.tolist(), dist_coeffs=D.ravel().tolist(),
        distortion_model="plumb_bob",
        new_camera_matrix=newK.tolist(), alpha=res["alpha"], roi_valida=res["roi"],
        erros_por_imagem={Path(c["path"]).name: (None if np.isnan(c.get("erro", np.nan))
                                                 else round(c["erro"], 4))
                          for c in capturas if not c["rejeitada"]},
        rejeitadas=[Path(c["path"]).name for c in capturas if c["rejeitada"]],
    )
    (pasta / "calibracao.json").write_text(json.dumps(info, indent=2), encoding="utf-8")

    print(f"\nResultados gravados em: {pasta.resolve()}")
    print(f"   - {fich_yaml}  (copiar para config/ no repositório)")
    print("   - calibracao.json    (relatório: RMS, erro por foto, rejeitadas)")
    print("\n# ---- valores da calibração ----")
    print("K = np.array(" + np.array2string(K, separator=", ", precision=8) + ")")
    print("D = np.array(" + np.array2string(D.ravel(), separator=", ", precision=8) + ")")
    print("# ------------------------------------\n")


# --------------------------------------------------------------------------- #
# Revisão das capturas
# --------------------------------------------------------------------------- #
def ajustar(img, max_w=1280, max_h=800):
    h, w = img.shape[:2]
    s = min(1.0, max_w / w, max_h / h)
    return (cv2.resize(img, (int(w * s), int(h * s))), s) if s < 1 else (img, 1.0)


def texto(img, linhas, org=(10, 25), cor=(255, 255, 255), escala=0.6):
    x, y = org
    fonte = cv2.FONT_HERSHEY_SIMPLEX
    for ln in linhas:
        (tw, th), base = cv2.getTextSize(ln, fonte, escala, 1)
        y0, y1 = max(y - th - 4, 0), min(y + base + 2, img.shape[0])
        x1 = min(x + tw + 4, img.shape[1])
        if y1 > y0 and x1 > x - 4:
            roi = img[y0:y1, max(x - 4, 0):x1]
            roi[:] = (roi * 0.35).astype(img.dtype)  # faixa escura de fundo
        cv2.putText(img, ln, (x, y), fonte, escala, cor, 1, cv2.LINE_AA)
        y += int(28 * escala / 0.6)


def rever(capturas, padrao, erro_max):
    """Janela de revisão. Devolve (mudou, usou_o): se alguma rejeição mudou e se o O foi usado."""
    if not capturas:
        return False, False
    antes = [c["rejeitada"] for c in capturas]
    usou_o = False
    i = 0
    cache = {}
    ajuda = "<-/A ant | ->/D/ESPACO seg | X/Del rejeitar | O rejeitar outliers | ENTER/ESC terminar"
    while True:
        c = capturas[i]
        if c["path"] not in cache:
            if len(cache) > 20:
                cache.clear()
            cache[c["path"]] = cv2.imread(c["path"])
        img = cache[c["path"]].copy()
        cv2.drawChessboardCorners(img, padrao, c["corners"], True)
        if c.get("reproj") is not None:
            for p in c["reproj"].reshape(-1, 2):
                cv2.circle(img, (int(round(p[0])), int(round(p[1]))), 3, (0, 0, 255), 1)
        vis, _ = ajustar(img)
        erros = np.array([x.get("erro", np.nan) for x in capturas], float)
        e = c.get("erro", np.nan)
        estado = "REJEITADA" if c["rejeitada"] else "MANTIDA"
        cor = (60, 60, 255) if c["rejeitada"] else (80, 255, 80)
        if c["rejeitada"]:
            cv2.rectangle(vis, (0, 0), (vis.shape[1] - 1, vis.shape[0] - 1), (0, 0, 255), 6)
        validas = sum(not x["rejeitada"] for x in capturas)
        lin = [f"{i+1}/{len(capturas)}  {Path(c['path']).name}  [{estado}]",
               (f"erro reproj.: {e:.3f} px   (media {np.nanmean(erros):.3f}, max {np.nanmax(erros):.3f})"
                if not np.isnan(e) else "erro reproj.: (calibre primeiro)"),
               f"validas: {validas}"]
        texto(vis, lin, cor=cor)
        texto(vis, [ajuda], org=(10, vis.shape[0] - 12), escala=0.45)
        # barra de erros
        if np.isfinite(erros).any():
            bw = max(2, (vis.shape[1] - 20) // len(capturas))
            emax = max(np.nanmax(erros), 1e-6)
            for j, x in enumerate(capturas):
                ej = x.get("erro", np.nan)
                if np.isnan(ej):
                    continue
                hb = int(40 * ej / emax)
                cb = (0, 0, 255) if x["rejeitada"] else ((0, 200, 255) if ej > erro_max else (0, 200, 0))
                x0 = 10 + j * bw
                cv2.rectangle(vis, (x0, vis.shape[0] - 40 - hb), (x0 + bw - 1, vis.shape[0] - 40), cb, -1)
                if j == i:
                    cv2.rectangle(vis, (x0, vis.shape[0] - 82), (x0 + bw - 1, vis.shape[0] - 40),
                                  (255, 255, 255), 1)
        cv2.imshow(JANELA_REV, vis)
        k = tecla(cv2.waitKeyEx(0))
        if k in K_DIR:
            i = (i + 1) % len(capturas)
        elif k in K_ESQ:
            i = (i - 1) % len(capturas)
        elif k in K_DEL:
            c["rejeitada"] = not c["rejeitada"]
        elif k == ord("o"):
            if not np.isfinite(erros).any():
                print("[revisao] ainda nao ha erros calculados.")
                continue
            m, s = np.nanmean(erros), np.nanstd(erros)
            lim = max(m + 2 * s, erro_max)
            n = 0
            for x in capturas:
                if x.get("erro", 0) > lim and not x["rejeitada"]:
                    x["rejeitada"] = True
                    n += 1
            usou_o = True
            print(f"[revisao] {n} outliers rejeitados (erro > {lim:.3f} px)")
        elif k in K_ENTER or k == K_ESC:
            break
        if cv2.getWindowProperty(JANELA_REV, cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyWindow(JANELA_REV)
    return antes != [c["rejeitada"] for c in capturas], usou_o


def flags_calib(args):
    """Como o '-k' do ROS: por omissão só k1 e k2 (k3 fixo a 0)."""
    flags = 0
    nk = getattr(args, "k_coeficientes", 2)
    if nk < 3:
        flags |= cv2.CALIB_FIX_K3
    if nk < 2:
        flags |= cv2.CALIB_FIX_K2
    if args.zero_tangente:
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
    return flags


def mover_rejeitadas(capturas):
    """Move as fotos rejeitadas para imagens/rejeitadas (não apaga nada)."""
    for c in capturas:
        p = Path(c["path"])
        if c["rejeitada"] and p.parent.name != "rejeitadas" and p.exists():
            rej_dir = p.parent / "rejeitadas"
            rej_dir.mkdir(exist_ok=True)
            novo = rej_dir / p.name
            p.replace(novo)
            c["path"] = str(novo)


# --------------------------------------------------------------------------- #
# Modo ao vivo
# --------------------------------------------------------------------------- #
def desenhar_cobertura(vis, cobertura):
    gx, gy = GRELHA_COB
    h, w = vis.shape[:2]
    over = vis.copy()
    for yy in range(gy):
        for xx in range(gx):
            n = cobertura[yy, xx]
            if n > 0:
                a = min(1.0, 0.35 + 0.1 * n)
                cor = (0, int(255 * a), 0)
                cv2.rectangle(over, (xx * w // gx, yy * h // gy),
                              ((xx + 1) * w // gx, (yy + 1) * h // gy), cor, -1)
    cv2.addWeighted(over, 0.25, vis, 0.75, 0, vis)
    for xx in range(1, gx):
        cv2.line(vis, (xx * w // gx, 0), (xx * w // gx, h), (90, 90, 90), 1)
    for yy in range(1, gy):
        cv2.line(vis, (0, yy * h // gy), (w, yy * h // gy), (90, 90, 90), 1)


def atualizar_cobertura(cobertura, corners, w, h):
    gx, gy = GRELHA_COB
    for x, y in corners.reshape(-1, 2):
        cx = int(np.clip(x / w * gx, 0, gx - 1))
        cy = int(np.clip(y / h * gy, 0, gy - 1))
        cobertura[cy, cx] += 1


def caixa_aviso(vis, linhas, cor=(0, 200, 255)):
    """Caixa grande ao centro do ecrã."""
    h, w = vis.shape[:2]
    y0, y1 = h // 2 - 20 - 18 * len(linhas), h // 2 + 20 + 18 * len(linhas)
    roi = vis[max(y0, 0):min(y1, h), :]
    roi[:] = (roi * 0.25).astype(vis.dtype)
    cv2.rectangle(vis, (4, max(y0, 0)), (w - 5, min(y1, h) - 1), cor, 3)
    y = y0 + 40
    for ln in linhas:
        (tw, _), _ = cv2.getTextSize(ln, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.putText(vis, ln, (max((w - tw) // 2, 10), y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    cor, 1, cv2.LINE_AA)
        y += 32


def modo_ao_vivo(args):
    padrao = (args.colunas, args.linhas)
    sessao = Path(args.saida) / f"camara{args.camara}_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    pasta_img = sessao / "imagens"
    pasta_img.mkdir(parents=True, exist_ok=True)

    fonte = FonteUVC(args.cam, args.largura, args.altura, args.fps, args.mjpg)

    cv2.namedWindow(JANELA, cv2.WINDOW_AUTOSIZE)
    capturas, descritores = [], []
    cobertura = np.zeros((GRELHA_COB[1], GRELHA_COB[0]), int)
    auto = args.auto
    undist = False
    mapas = None
    # estado da calibração
    res = None          # última calibração calculada
    res_atual = False   # True se 'res' foi calculada com o conjunto atual de fotos
    o_feito = False     # True se o O (remover outliers) foi usado desde a última foto nova
    limpo = False       # True se o C foi feito DEPOIS do O, sem mudanças desde então
    aviso_q = False     # aviso "vai gravar sem remover outliers" ativo
    gravado = False
    mensagem = None     # (linhas, cor, instante_fim)
    ant = None
    estavel = 0
    t_ultima = 0.0
    flash = 0
    falhas = 0
    fps_t, fps_n, fps_v = time.time(), 0, 0.0
    tamanho = None

    def validas():
        return sum(not c["rejeitada"] for c in capturas)

    def pronto():
        """Critério do ROS (barras cheias ou 40 fotos) e o mínimo de fotos válidas."""
        return progresso_ros(descritores)[1] and validas() >= MIN_IMAGENS

    def msg_faltam():
        msg(["Ainda nao ha fotos suficientes:",
             f"encha as barras X, Y, Tam, Incl (ou tire {MIN_FOTOS_ROS} fotos)."], seg=4)

    def msg(linhas, cor=(0, 200, 255), seg=3.0):
        nonlocal mensagem
        mensagem = (linhas, cor, time.time() + seg)

    def calibrar_agora():
        nonlocal res, res_atual, mapas
        r = calibrar(capturas, padrao, args.quadrado, tamanho, flags_calib(args), args.alpha)
        if r is not None:
            res, res_atual = r, True
            mapas = cv2.initUndistortRectifyMap(res["K"], res["D"], None, res["newK"],
                                                res["tamanho"], cv2.CV_16SC2)
        return r

    print("\nOrdem recomendada: tirar fotos -> R (rever) -> O (remover outliers) -> ENTER"
          " -> C (calibrar) -> U (verificar) -> Q (gravar e sair)")
    print("Teclas: ESPACO foto | A auto | R rever | C calibrar | U ver corrigida "
          "| Q gravar e sair | ESC sair sem gravar\n")

    try:
        while True:
            ok, frame = fonte.ler()
            if not ok or frame is None:
                falhas += 1
                if falhas > 50:
                    print("[!] A câmara deixou de enviar imagens.")
                    break
                cv2.waitKey(10)
                continue
            falhas = 0
            h, w = frame.shape[:2]
            if tamanho is None:
                tamanho = (w, h)
            elif tamanho != (w, h):
                print("[!] A resolução mudou durante a sessão; ignorar frame.")
                continue

            esc = min(1.0, args.preview_largura / w)
            peq = cv2.resize(frame, None, fx=esc, fy=esc) if esc < 1 else frame
            gpeq = cv2.cvtColor(peq, cv2.COLOR_BGR2GRAY)
            achou, cs = detetar_xadrez(gpeq, padrao, rapido=True)
            na_borda = achou and perto_da_borda(cs / esc, w, h)
            if na_borda:  # como no ROS: cantos a menos de 8 px da borda não contam
                achou = False

            # estabilidade
            if achou and ant is not None and ant.shape == cs.shape:
                desloc = float(np.mean(np.linalg.norm(cs - ant, axis=2)))
                estavel = estavel + 1 if desloc < args.limiar_movimento else 0
            else:
                estavel = 0
            ant = cs if achou else None

            novidade = None
            if achou:
                d = parametros_ros(cs / esc, padrao, w, h)
                novidade = (min(distancia_ros(d, x) for x in descritores)
                            if descritores else 1e9)

            # decidir captura (não captura com o aviso do Q no ecrã)
            pedir = False
            k = tecla(cv2.waitKeyEx(1))
            if aviso_q and k != -1 and k != ord("q"):
                aviso_q = False  # qualquer outra tecla cancela o aviso
                print("Gravação cancelada.")
            if k == ord(" ") and achou:
                pedir = True
            elif (auto and achou and not aviso_q and estavel >= args.frames_estaveis
                  and novidade > args.novidade and time.time() - t_ultima > args.intervalo):
                pedir = True

            if pedir:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                okf, cf = detetar_xadrez(gray, padrao, rapido=False)
                if not okf:  # usar a deteção do preview refinada na resolução total
                    cf = (cs / esc).astype(np.float32)
                    cf = cv2.cornerSubPix(gray, cf, janela_subpix(cf, padrao), (-1, -1), CRIT_SUBPIX)
                nt = nitidez(gray, cf)
                if perto_da_borda(cf, w, h):
                    print("[x] xadrez demasiado perto da borda, ignorado")
                elif nt < args.nitidez_min:
                    print(f"[x] imagem tremida (nitidez {nt:.0f} < {args.nitidez_min}), ignorada")
                else:
                    caminho = pasta_img / f"img_{len(capturas):03d}.png"
                    cv2.imwrite(str(caminho), frame)
                    capturas.append(dict(path=str(caminho), corners=cf, tamanho=(w, h),
                                         rejeitada=False))
                    descritores.append(parametros_ros(cf, padrao, w, h))
                    atualizar_cobertura(cobertura, cf, w, h)
                    res_atual = o_feito = limpo = False  # foto nova: repetir R -> O -> C
                    t_ultima = time.time()
                    estavel = 0
                    flash = 4
                    print(f"[+] captura {len(capturas)}: {caminho.name}  (nitidez {nt:.0f})")

            # ---------- teclas ----------
            if k == ord("a"):
                auto = not auto
                print(f"captura automática: {'ON' if auto else 'OFF'}")
            elif k == ord("u"):
                if res is None:
                    msg(["Calibre primeiro (C)."])
                else:
                    undist = not undist
            elif k == ord("r"):
                if not pronto():
                    msg_faltam()
                else:
                    if auto:  # para não tirar fotos sem querer enquanto revê/calibra
                        auto = False
                        print("Captura automática desligada (A para voltar a tirar fotos).")
                    if not res_atual:  # calcula os erros de cada foto para a revisão
                        calibrar_agora()
                    mudou, usou_o = rever(capturas, padrao, args.erro_max)
                    if usou_o:
                        o_feito = True
                    if mudou:
                        res_atual = False
                    limpo = False
                    if o_feito:
                        msg(["Outliers tratados. Agora prima C para recalibrar sem eles.",
                             "(A = voltar a tirar fotos)"], (80, 255, 80), 5)
                    else:
                        msg(["Na revisao use O para remover os outliers.",
                             "(A = voltar a tirar fotos)"], seg=5)
            elif k == ord("c"):
                if not pronto():
                    msg_faltam()
                elif calibrar_agora() is not None:
                    limpo = o_feito
                    if limpo:
                        msg([f"Calibrado sem outliers: RMS {res['rms']:.3f} px.",
                             "U para verificar, Q para gravar."], (80, 255, 80), 6)
                    else:
                        msg([f"Calibrado: RMS {res['rms']:.3f} px (outliers NAO removidos).",
                             "Para os remover: R -> O -> ENTER -> C."], seg=6)
            elif k == ord("q"):
                if not capturas:
                    print("Sem fotos: nada a gravar.")
                    break
                if not pronto():
                    msg(["Ainda nao ha fotos suficientes para gravar:",
                         f"encha as barras X, Y, Tam, Incl (ou tire {MIN_FOTOS_ROS} fotos).",
                         "ESC sai sem gravar."], (60, 60, 255), 5)
                elif limpo and res_atual:
                    gravar_resultados(res, capturas, args, sessao)
                    mover_rejeitadas(capturas)
                    gravado = True
                    break
                elif not aviso_q:
                    aviso_q = True
                    print("[AVISO] " + ("Falta o C depois de remover os outliers." if o_feito
                                        else "Vai gravar SEM remover os outliers.")
                          + " Prima Q de novo para confirmar, ou outra tecla para cancelar.")
                else:  # segundo Q: gravar a calibração assim mesmo
                    if not res_atual:
                        calibrar_agora()
                    print("[AVISO] " + ("Recalibrada sem as fotos rejeitadas e gravada." if o_feito
                                        else "Calibração gravada SEM remover os outliers."))
                    gravar_resultados(res, capturas, args, sessao)
                    mover_rejeitadas(capturas)
                    gravado = True
                    break
            elif k == K_ESC:
                break
            if cv2.getWindowProperty(JANELA, cv2.WND_PROP_VISIBLE) < 1:
                break

            # ---------- visualização ----------
            lin_err = None
            if undist and mapas is not None:
                fu = cv2.remap(frame, mapas[0], mapas[1], cv2.INTER_LINEAR)
                vis = cv2.resize(fu, (peq.shape[1], peq.shape[0])) if esc < 1 else fu
                if achou:  # 'lin.' do ROS: quão retas ficam as linhas do xadrez corrigido
                    pu = cv2.undistortPoints((cs / esc).astype(np.float64), res["K"], res["D"],
                                             P=res["newK"])
                    lin_err = erro_linearidade(pu, padrao)
                    cv2.drawChessboardCorners(vis, padrao, (pu * esc).astype(np.float32), True)
            else:
                vis = peq.copy()
                desenhar_cobertura(vis, cobertura)
                if achou:
                    cv2.drawChessboardCorners(vis, padrao, cs, True)

            fps_n += 1
            if time.time() - fps_t > 1:
                fps_v, fps_n, fps_t = fps_n / (time.time() - fps_t), 0, time.time()

            cob = 100.0 * np.count_nonzero(cobertura) / cobertura.size
            prog = min(1.0, estavel / max(1, args.frames_estaveis))
            barras, suficiente = progresso_ros(descritores)
            lin = [f"Camara {args.camara}  Capturas: {len(capturas)}  Cobertura: {cob:.0f}%"
                   f"  Auto: {'ON' if auto else 'OFF'}"
                   f"  {w}x{h}  {fps_v:.0f} fps",
                   ("Xadrez: DETETADO" if achou else
                    "Xadrez: demasiado perto da borda" if na_borda else "Xadrez: nao detetado")
                   + (f"  pose nova: {'sim' if novidade > args.novidade else 'nao (mude a pose)'}"
                      if achou else ""),
                   ("Fotos suficientes: pode rever (R) e calibrar (C)" if suficiente
                    else "Encha as barras X, Y, Tam, Incl (ou "
                         f"{MIN_FOTOS_ROS} fotos)"),
                   f"Passos:  R+O outliers [{'ok' if o_feito else '--'}]"
                   f"   C calibrar [{'ok' if limpo and res_atual else '--'}]   Q gravar"]
            if res is not None:
                lin.append(f"Calibrado: RMS {res['rms']:.3f} px"
                           + ("" if res_atual else " (desatualizado, prima C)")
                           + ("  [VISTA CORRIGIDA]" if undist else "")
                           + (f"  lin.: {lin_err:.2f} px" if lin_err is not None else ""))
            texto(vis, lin, cor=(80, 255, 80) if achou else (80, 180, 255))
            if achou and auto:
                bw = int(200 * prog)
                cv2.rectangle(vis, (10, vis.shape[0] - 45), (210, vis.shape[0] - 35), (80, 80, 80), 1)
                cv2.rectangle(vis, (10, vis.shape[0] - 45), (10 + bw, vis.shape[0] - 35),
                              (0, 255, 0) if prog >= 1 else (0, 200, 255), -1)
            # barras X / Y / Tam / Incl (como no ROS), canto inferior direito
            x0b, y0b = vis.shape[1] - 160, vis.shape[0] - 40 - 4 * 16
            for n_b, (nome_b, v_b) in enumerate(zip(["X", "Y", "Tam", "Incl"], barras)):
                yb = y0b + n_b * 16
                texto(vis, [nome_b], org=(x0b - 42, yb + 10), escala=0.4)
                cv2.rectangle(vis, (x0b, yb), (x0b + 140, yb + 11), (80, 80, 80), 1)
                cv2.rectangle(vis, (x0b, yb), (x0b + int(140 * v_b), yb + 11),
                              (0, 220, 0) if v_b >= 1 else (0, 200, 255), -1)
            texto(vis, ["ESPACO foto | A auto | R rever (O) | C calibrar | U corrigida | Q gravar | ESC sair"],
                  org=(10, vis.shape[0] - 12), escala=0.42)
            if aviso_q:
                if o_feito:
                    caixa_aviso(vis, ["AVISO: falta o C depois de remover os outliers",
                                      "Q de novo = recalibrar sem eles e gravar",
                                      "outra tecla = cancelar"])
                else:
                    caixa_aviso(vis, ["AVISO: vai gravar SEM remover os outliers",
                                      "(falta R -> O -> ENTER -> C)",
                                      "Q de novo = gravar assim  |  outra tecla = cancelar"])
            elif mensagem is not None:
                if time.time() < mensagem[2]:
                    caixa_aviso(vis, mensagem[0], mensagem[1])
                else:
                    mensagem = None
            if flash > 0:
                cv2.rectangle(vis, (0, 0), (vis.shape[1] - 1, vis.shape[0] - 1), (255, 255, 255), 12)
                flash -= 1
            cv2.imshow(JANELA, vis)
    finally:
        fonte.fechar()
        cv2.destroyAllWindows()
    if not gravado:
        print("Saiu SEM gravar a calibração.")
    print(f"Imagens da sessão em: {pasta_img.resolve()}")


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="Calibração da Orbbec Astra com xadrez (sem ROS).",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    g = ap.add_argument_group("câmara")
    g.add_argument("--camara", help="designação da câmara física, ex.: 5 ou 8 (obrigatório)")
    g.add_argument("--cam", type=int, default=0, help="índice da câmara UVC")
    g.add_argument("--largura", type=int, default=640)
    g.add_argument("--altura", type=int, default=480)
    g.add_argument("--fps", type=int, default=30)
    g.add_argument("--mjpg", action="store_true", help="forçar MJPG (resoluções altas em UVC)")
    g.add_argument("--listar", action="store_true", help="listar câmaras UVC e sair")

    g = ap.add_argument_group("xadrez")
    g.add_argument("--colunas", type=int, default=9, help="cantos INTERNOS na horizontal")
    g.add_argument("--linhas", type=int, default=6, help="cantos INTERNOS na vertical")
    g.add_argument("--quadrado", type=float, default=25.0, help="lado do quadrado")
    g.add_argument("--unidade", default="mm", help="unidade do --quadrado (só informativo)")

    g = ap.add_argument_group("captura automática")
    g.add_argument("--auto", action="store_true", help="começar já em modo automático")
    g.add_argument("--frames-estaveis", type=int, default=8, help="frames parado antes de capturar")
    g.add_argument("--limiar-movimento", type=float, default=1.5, help="px (preview) p/ considerar parado")
    g.add_argument("--novidade", type=float, default=0.2,
                   help="diferença mínima (X+Y+Tam+Incl) para aceitar uma pose nova (ROS: 0.2)")
    g.add_argument("--intervalo", type=float, default=1.0, help="s mínimos entre capturas")
    g.add_argument("--nitidez-min", type=float, default=30.0, help="variância Laplaciano mínima")
    g.add_argument("--preview-largura", type=int, default=960, help="largura máx. da janela")

    g = ap.add_argument_group("calibração")
    g.add_argument("--k-coeficientes", type=int, choices=[1, 2, 3], default=2,
                   help="nº de coeficientes radiais estimados; 2 = k1,k2 com k3=0 (igual ao ROS)")
    g.add_argument("--zero-tangente", action="store_true", help="p1=p2=0")
    g.add_argument("--alpha", type=float, default=0.0, help="0 recorta bordas, 1 mantém todos os pixels")
    g.add_argument("--erro-max", type=float, default=1.0, help="px; destacar fotos acima disto")
    g.add_argument("--saida", default="calibracoes", help="pasta base dos resultados")
    args = ap.parse_args()

    print(f"OpenCV {cv2.__version__} | detetor SB: {'sim' if HAS_SB else 'nao'}")
    if args.listar:
        listar_cameras()
        return
    if not args.camara:
        sys.exit("Erro: indique qual é a câmara física, por exemplo --camara 5 ou --camara 8.")
    modo_ao_vivo(args)


if __name__ == "__main__":
    main()