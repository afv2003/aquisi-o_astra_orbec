"""Testes da estrada (Frenet) e da ocupação da faixa (correm sem ROS: python3 -m pytest src/va_manobras)."""
import numpy as np

from va_manobras.estrada import Estrada, pista_oval
from va_manobras.faixa import ocupacao_da_faixa


def curva(raio=30.0, comp=60.0):
    """Linha de centro no base_link: começa 5 m atrás do carro e curva para a esquerda."""
    s = np.arange(-5.0, comp, 1.0)
    a = np.clip(s, 0, None) / raio
    x = np.where(s < 0, s, raio * np.sin(a))
    y = np.where(s < 0, 0.0, raio - raio * np.cos(a))
    return Estrada(x, y)


def test_frenet_numa_reta():
    e = Estrada([0, 100], [0, 0])
    s, d = e.para_frenet([10.0, 50.0], [2.0, -1.5])
    assert np.allclose(s, [10, 50]) and np.allclose(d, [2.0, -1.5])


def test_ida_e_volta_numa_curva():
    e = curva()
    x, y = e.de_frenet([20.0, 40.0], [1.2, -0.8])
    s, d = e.para_frenet(x, y)
    assert np.allclose(s, [20, 40], atol=0.05) and np.allclose(d, [1.2, -0.8], atol=0.05)


def test_pista_oval_fechada():
    e = pista_oval()
    assert e.fechada and abs(e.comprimento - (240 + 2 * np.pi * 35)) < 2.0
    x, y = e.de_frenet(e.comprimento + 10.0, 0.0)        # s volta ao início
    assert np.hypot(x[0] - 10.0, y[0]) < 0.1


def test_carro_na_minha_faixa_em_curva():
    e = curva()
    xo, yo = e.de_frenet(np.full(5, 30.0), np.linspace(-0.8, 0.8, 5))   # traseira de um carro 25 m à frente (s = 0 é 5 m atrás)
    oc = ocupacao_da_faixa(xo, yo, e, x_frente=3.0)
    assert not oc.livre and abs(oc.distancia - 22.0) < 0.5


def test_carro_da_outra_faixa_nao_conta_com_frenet_mas_conta_a_direito():
    e = curva()
    xo, yo = e.de_frenet(np.full(5, 25.0), 3.5 + np.linspace(-0.8, 0.8, 5))   # na faixa da esquerda
    assert ocupacao_da_faixa(xo, yo, e, modo='frenet').livre
    # visto "a direito" (sem estrada), um carro da faixa ao lado parece estar à frente? depende da curva:
    # aqui está em y ~ 13 m, fora do corredor reto; mas a berma da direita entra no corredor reto
    xb, yb = e.de_frenet(np.arange(5.0, 40.0, 0.5), -2.0)
    assert ocupacao_da_faixa(xb, yb, e, modo='frenet').livre
    assert not ocupacao_da_faixa(xb, yb, e, modo='reto').livre
