# -*- coding: utf-8 -*-
"""
Testes da parte 2: os avisos com a estrada (/estrada/centro, contrato da aula 11) em coordenadas de Frenet,
e a gravação e leitura dos outros sensores (GNSS).

Cena sintética numa CURVA atrás do carro (a estrada vira para a esquerda: centro da faixa em
y = K_CURVA * x^2 para x < 0), projetada nas duas câmaras como em test_avisos.py. Sem a estrada, os avisos
supõem a estrada reta junto ao carro e erram na curva; com ela, seguem a faixa.

    python3 -m pytest -q src/va_g05/test          (no Docker: /ws/src; README, secção 1.7)
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    import va_g05  # noqa: F401  (no Docker: instalado pelo colcon)
except ImportError:                                   # fora do Docker: a partir do código-fonte
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from test_avisos import ALTURA, K, LARGURA, MONTAGEM, UU, VV, _projetar  # noqa: E402
from va_g05 import aviso_saida_faixa, decisao, estrada_local, gravacao_online, leitura_bag, outros_sensores  # noqa: E402

K_CURVA = 0.012
LARGURA_FAIXA = 3.5


def centro(x):
    x = np.asarray(x, float)
    return np.where(x < 0, K_CURVA * x ** 2, 0.0)


def pontos_estrada():
    xs = np.arange(-30.0, 50.5, 1.0)
    return xs.tolist(), centro(xs).tolist()


def _cena(d, t_ns, veiculos=(), com_estrada=True):
    """veiculos: [(x da frente, d)]: d = afastamento do centro da faixa do carro (0 = a mesma faixa)."""
    if com_estrada:
        d.atualizar_estrada(*pontos_estrada(), t_ns)
    res = None
    for cam in ("5", "8"):
        geo = d.grelha.geometria(cam, K)[0]
        x, y, ok = geo.pixel_para_chao(UU, VV)
        estrada = ok & (np.abs(y - centro(x)) < 5.25)
        caixas = []
        for xf, dv in veiculos:
            yc = float(centro(xf - 2.25)) + dv
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
        res = d.atualizar(cam, K, LARGURA, ALTURA, t_ns, estrada, caixas, [])
    return res


def _correr(veiculos, com_estrada, n=3):
    d = decisao.DecisaoLocal(MONTAGEM)
    r = None
    for i in range(n):
        r = _cena(d, 10**18 + int(i * 0.1e9), veiculos, com_estrada)
    return r


# ------------------------------------------------------------------ mudança de faixa e colisão na curva
def test_curva_veiculo_na_propria_faixa():
    """Carro atrás, na NOSSA faixa, já na curva: sem a estrada parece estar na faixa da esquerda."""
    veic = [(-14.0, 0.0)]
    sem = _correr(veic, com_estrada=False)
    com = _correr(veic, com_estrada=True)
    assert sem["faixa"]["esquerda"]["estado"] == "OCUPADA"            # erro de supor a estrada reta
    assert com["faixa"]["esquerda"]["faixa"] == "estrada"
    assert com["faixa"]["esquerda"]["estado"] != "OCUPADA"
    # colisão: o objeto com posição no chão (a outra câmara vê-o cortado pela borda: sem posição) está atrás,
    # na própria faixa; sem a estrada, à esquerda
    assert {o["zona"] for o in com["colisao"]["objetos"] if o["x"] is not None} == {"atras"}
    assert {o["zona"] for o in sem["colisao"]["objetos"] if o["x"] is not None} == {"esquerda"}


def test_curva_veiculo_na_faixa_da_esquerda():
    """Carro atrás, na faixa da esquerda, na curva: sem a estrada fica fora da zona; com ela, OCUPADA."""
    veic = [(-14.0, LARGURA_FAIXA)]
    sem = _correr(veic, com_estrada=False)
    com = _correr(veic, com_estrada=True)
    assert sem["faixa"]["esquerda"]["estado"] != "OCUPADA"
    assert com["faixa"]["esquerda"]["estado"] == "OCUPADA"
    assert len(com["faixa"]["esquerda"]["zona"]) > 4                    # a zona segue a curva


def test_estrada_antiga_e_ignorada():
    d = decisao.DecisaoLocal(MONTAGEM)
    d.atualizar_estrada(*pontos_estrada(), 10**18)
    r = _cena(d, 10**18 + int(3e9), [], com_estrada=False)              # 3 s depois: já não serve
    assert r["estrada"] is None and r["faixa"]["esquerda"]["faixa"] != "estrada"


def test_modo_estrada_sem_estrada():
    m = dict(MONTAGEM)
    m["aviso_faixa"] = {**(MONTAGEM.get("aviso_faixa") or {}), "modo_faixa": "estrada"}
    d = decisao.DecisaoLocal(m)
    r = _cena(d, 10**18, [], com_estrada=False)
    assert {i["estado"] for i in r["faixa"].values()} == {"SEM_INFORMACAO"}
    assert {i.get("motivo") for i in r["faixa"].values()} == {"sem estrada"}


# ------------------------------------------------------------------ saída de faixa pela estrada
def test_saida_pela_estrada_sem_linhas():
    """Sem linhas detetadas, a borda da faixa vem da estrada: carro 0,5 m à esquerda do centro."""
    e = estrada_local.EstradaLocal([-30, 50], [-0.5, -0.5], 10**18)
    aviso = aviso_saida_faixa.AvisoSaida(aviso_saida_faixa.parametros(MONTAGEM), (-0.6, 3.0, 1.5), LARGURA_FAIXA)
    sem_linhas = SimpleNamespace(_recente=lambda chave, t: None)
    r = aviso.avaliar(sem_linhas, 10**18, None, e)
    assert r["esquerda"]["fonte"] == "estrada"
    assert abs(r["esquerda"]["distancia_m"] - 0.5) < 0.05                # 1,25 - 0,75
    assert abs(r["direita"]["distancia_m"] - 1.5) < 0.05                 # 2,25 - 0,75
    assert aviso.avaliar(sem_linhas, 10**18, None, None)["esquerda"]["estado"] == "SEM_LINHA"


def test_frenet_continua_reta_fora_da_estrada():
    e = estrada_local.EstradaLocal([-5, 0, 20], [0, 0, 0], 1)
    s, d = e.frenet([-12.0, 30.0], [1.0, -2.0])
    assert np.allclose(s, [-12, 30]) and np.allclose(d, [1, -2])


# ------------------------------------------------------------------ outros sensores: GNSS no bag
def test_configurados():
    assert set(outros_sensores.configurados("")) == {"/fix", "/odom", "/tf", "/estrada/centro"}
    assert outros_sensores.configurados("nenhum") == {}
    assert outros_sensores.configurados("/gnss/fix=sensor_msgs/msg/NavSatFix /odom") == {
        "/gnss/fix": "sensor_msgs/msg/NavSatFix", "/odom": "nav_msgs/msg/Odometry"}


def test_gnss_gravado_e_lido(tmp_path):
    from mcap_ros2._dynamic import serialize_dynamic
    tipo = outros_sensores.TIPO_GNSS
    ser = serialize_dynamic(tipo, outros_sensores.DEFINICAO_NAVSATFIX)[tipo]
    bag = tmp_path / "s_0.mcap"
    g = gravacao_online.GravadorTrechos(bag, ["5"], outros_sensores.canais_para_gravar({"/fix": tipo}))
    g.iniciar_trecho()
    t0 = g.inicio_ns + 1_000_000_000
    for i in range(5):
        st = -1 if i == 2 else 0                                         # uma sem fix: ignorada
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="gnss"),
                              status=SimpleNamespace(status=st, service=1), latitude=40.63 + i * 1e-5,
                              longitude=-8.65, altitude=10.0, position_covariance=[0.0] * 9,
                              position_covariance_type=0)
        g.mensagem(outros_sensores.chave("/fix"), ser(msg), t0 + i * 100_000_000)
    # uma imagem, para o bag não ser apagado no fim (sem imagens não fica bag)
    img = gravacao_online.cdr_imagem_comprimida(t0 // 10**9, t0 % 10**9, "camara5", gravacao_online.FORMATO_JPEG,
                                                b"\xff\xd8\xff\xd9")
    g.imagem("5", img)
    g.fechar()
    assert leitura_bag.tipos_do_bag(bag)["/fix"] == tipo
    pos = leitura_bag.ler_gnss(bag)
    assert len(pos) == 4 and pos[0][1] == 40.63 and pos[-1][1] > pos[0][1]
