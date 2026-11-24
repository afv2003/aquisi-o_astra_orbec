# -*- coding: utf-8 -*-
"""
Testes da decisão local (grelha local e os três avisos) com cenas sintéticas.

Não há câmaras no CI (nem simulador de câmaras): as "deteções" de cada imagem são geradas a partir de
uma cena no chão (linhas da estrada a ±1,75 m e ±5,25 m, veículos como caixas de 4,5 x 1,8 x 1,5 m),
projetada nas duas câmaras com a montagem e a calibração do config/ (as mesmas do online e do offline).
O que se testa é o que vem depois dos modelos: decisao.DecisaoLocal (grelha, faixas, seguimento, TTC,
avisos), que é o mesmo código no online (processar_tempo_real.py) e no offline (processar_bag.py).

    python3 -m pytest -q src/va_g05/test          (no Docker: /ws/src; README, secção 1.7)
"""
import sys
from pathlib import Path

import numpy as np
import pytest

try:
    import va_g05  # noqa: F401  (no Docker: instalado pelo colcon)
except ImportError:                                   # fora do Docker: a partir do código-fonte
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from va_g05 import aviso_colisao, aviso_faixa, aviso_saida_faixa, caminhos, decisao, geometria, leitura_bag

LARGURA, ALTURA = 640, 480
MONTAGEM = geometria.carregar_montagem()
K = geometria.matriz_intrinseca(leitura_bag.carregar_calibracao_yaml(caminhos.calibracao(5)), LARGURA, ALTURA, True)
VV, UU = np.mgrid[0:ALTURA, 0:LARGURA]


def _projetar(geo, x, y, z):
    p = np.stack([np.asarray(x, float) - geo.t[0], np.asarray(y, float) - geo.t[1],
                  np.asarray(z, float) - geo.t[2]], -1)
    c = p @ geo.r_otico
    uvw = c @ geo.k.T
    return uvw[..., 0] / c[..., 2], uvw[..., 1] / c[..., 2], c[..., 2] > 0.1


def _cena(d, t_ns, desvio=0.0, veiculos=(), linhas=True):
    """desvio: carro deslocado para a esquerda (m); veiculos: [(x da frente, y do centro)] no base_link."""
    res = None
    for cam in ("5", "8"):
        geo = d.grelha.geometria(cam, K)[0]
        x, y, ok = geo.pixel_para_chao(UU, VV)
        estrada = ok & (y < 5.25 - desvio) & (y > -5.25 - desvio)
        caixas = []
        for xf, yc in veiculos:
            xs = np.array([xf, xf, xf, xf, xf - 4.5, xf - 4.5])
            ys = np.array([yc - .9, yc + .9, yc - .9, yc + .9, yc - .9, yc + .9])
            u, v, frente = _projetar(geo, xs, ys, np.array([0, 0, 1.5, 1.5, 0, 0]))
            if not frente.all():
                continue
            x1, x2, y1, y2 = u.min(), u.max(), v.min(), v.max()
            if x2 < 0 or x1 > LARGURA - 1 or y2 < 0 or y1 > ALTURA - 1:
                continue
            x1, x2, y1, y2 = max(x1, 0), min(x2, LARGURA - 1), max(y1, 0), min(y2, ALTURA - 1)
            if x2 - x1 < 8:
                continue
            caixas.append((x1, y1, x2, y2, "car", 0.9))
            estrada &= ~((UU >= x1) & (UU <= x2) & (VV >= y1) & (VV <= y2))
        lin = []
        if linhas:
            for yl in (1.75, 5.25, -1.75, -5.25):
                xs = np.linspace(-22, 0, 30)
                ys = np.full_like(xs, yl - desvio)
                u, v, frente = geo.chao_para_pixel(xs, ys)
                sel = frente & (u >= 0) & (u < LARGURA) & (v >= 0) & (v < ALTURA) & (geo.distancia(xs, ys) >= 2.5)
                if sel.sum() >= 3:
                    lin.append(np.stack([u[sel], v[sel]], 1))
        res = d.atualizar(cam, K, LARGURA, ALTURA, t_ns, estrada, caixas, lin)
    return res


def _correr(n, cena, modo_faixa=None, passo=0.1):
    m = dict(MONTAGEM)
    if modo_faixa:
        m["aviso_faixa"] = {**(MONTAGEM.get("aviso_faixa") or {}), "modo_faixa": modo_faixa}
    d = decisao.DecisaoLocal(m)
    historico = []
    for i in range(n):
        historico.append(_cena(d, 10**18 + int(i * passo * 1e9), **cena(i * passo)))
    return historico


# ------------------------------------------------------------------ grelha local (contrato da aula 10)
def test_grelha_contrato():
    r = _correr(1, lambda t: dict(veiculos=[(-9, -3.5)]))[-1]
    g = r["grelha"]
    p = MONTAGEM["grelha"]
    assert g.dtype == np.int8
    assert g.shape == (round((p["y_max"] - p["y_min"]) / p["resolucao"]),
                       round((p["x_max"] - p["x_min"]) / p["resolucao"]))
    assert set(np.unique(g)) <= {-1, 0, 100}
    assert (g == 0).any() and (g == 100).any() and (g == -1).any()


# ------------------------------------------------------------------ mudança de faixa
@pytest.mark.parametrize("modo", ["auto", "centro"])
def test_faixa_direita_ocupada(modo):
    """Carro 0,6 m à esquerda do centro; veículo na faixa da direita, 9 m atrás."""
    r = _correr(3, lambda t: dict(desvio=0.6, veiculos=[(-9, -4.1)]), modo)[-1]
    assert r["faixa"]["direita"]["estado"] == "OCUPADA"
    assert r["faixa"]["esquerda"]["estado"] == "LIVRE"
    assert r["faixa"]["direita"]["faixa"] == ("linhas" if modo == "auto" else "centro")


def test_faixa_segunda_faixa_nao_conta_com_linhas():
    """Carro 0,5 m à esquerda; veículo na 2.ª faixa à esquerda: pelas linhas, a faixa ao lado está livre."""
    r = _correr(3, lambda t: dict(desvio=0.5, veiculos=[(-10, 6.0)]))[-1]
    assert r["faixa"]["esquerda"]["estado"] == "LIVRE"


def test_faixa_sem_linhas():
    sem = _correr(3, lambda t: dict(linhas=False), "linhas")[-1]
    assert {i["estado"] for i in sem["faixa"].values()} == {"SEM_INFORMACAO"}
    auto = _correr(3, lambda t: dict(linhas=False), "auto")[-1]
    assert {i["faixa"] for i in auto["faixa"].values()} == {"centro"}


def test_faixa_veiculo_a_aproximar():
    """Veículo na faixa da esquerda a aproximar-se 5 m/s: OCUPADA, com o tempo até chegar."""
    r = _correr(25, lambda t: dict(veiculos=[(-28 + 5 * t, 3.5)]))[-1]
    esq = r["faixa"]["esquerda"]
    assert esq["estado"] == "OCUPADA"
    assert "aproxima-se" in aviso_faixa.texto(esq)


# ------------------------------------------------------------------ saída de faixa
def test_saida_a_derivar():
    h = _correr(30, lambda t: dict(desvio=0.3 * t))
    assert h[0]["saida"]["esquerda"]["estado"] == "OK"
    assert h[-1]["saida"]["esquerda"]["estado"] in ("PERTO", "A_PISAR")
    assert h[-1]["saida"]["direita"]["estado"] == "OK"


def test_saida_com_faixa_ocupada_e_perigo():
    r = _correr(30, lambda t: dict(desvio=0.3 * t, veiculos=[(-4, 3.5 - 0.3 * t)]))[-1]
    assert r["saida"]["esquerda"].get("perigo")
    assert "FAIXA OCUPADA" in aviso_saida_faixa.texto(r["saida"]["esquerda"])


# ------------------------------------------------------------------ colisão
def test_colisao_veiculo_a_aproximar_atras():
    """Veículo na própria faixa a aproximar-se 8 m/s de 25 m: AVISO e depois PERIGO."""
    h = _correr(25, lambda t: dict(veiculos=[(-25 + 8 * t, 0.0)]))
    estados = [x["colisao"]["estado"] for x in h]
    assert "AVISO" in estados
    assert estados[-1] == "PERIGO"
    ttc = h[-1]["colisao"]["objeto"]["ttc_s"]
    assert 0.3 < ttc < 1.8           # real: 0,9 s (tremor das caixas e janela de 1,5 s)


@pytest.mark.parametrize("cena,final", [
    (lambda t: dict(veiculos=[(-8 - 4 * t, 0.0)]), "LIVRE"),             # afasta-se
    (lambda t: dict(veiculos=[(-25 + 8 * t, 7.0)]), "LIVRE"),            # aproxima-se duas faixas ao lado
    (lambda t: dict(veiculos=[(-7.0, 0.0)]), "ATENCAO"),                 # segue colado, sem se aproximar
])
def test_colisao_sem_aviso(cena, final):
    """Nunca AVISO nem PERIGO; no fim, LIVRE (ou ATENÇÃO, para quem segue colado atrás)."""
    h = _correr(20, cena)
    assert max(aviso_colisao.NIVEL[x["colisao"]["estado"]] for x in h) <= aviso_colisao.NIVEL["ATENCAO"]
    assert h[-1]["colisao"]["estado"] == final
