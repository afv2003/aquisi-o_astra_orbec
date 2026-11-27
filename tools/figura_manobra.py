"""Figuras e métricas de uma manobra, a partir de um bag (sem ROS). Aula 11.

    pip install mcap mcap-ros2-support matplotlib numpy
    python tools/figura_manobra.py data/<pasta_do_bag> --grupo g00
    python tools/figura_manobra.py data/<pasta_do_bag> --grupo g00 --instante 32

Gera em docs/img/:
  <bag>_manobra.png    velocidade, distância ao carro da frente (medida e, no simulador, verdadeira),
                       intervalo de tempo e estado da manobra ao longo do tempo
  <bag>_faixa.png      um instante: grelha local, linha de centro da estrada, faixa considerada e obstáculo
e escreve as métricas no terminal (distância mínima, intervalo de tempo, % do tempo em cada estado).
"""
import argparse
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon, Rectangle
from mcap_ros2.reader import read_ros2_messages

matplotlib.use('Agg')

VERDE, LIMA, VERMELHO, AZUL, CINZA = '#4a7c00', '#91d400', '#b91c1c', '#1d4ed8', '#5b6470'


def ler(bag, grupo):
    mcap = next(bag.glob('*.mcap')) if bag.is_dir() else bag
    g = f'/{grupo}'
    series = {k: [] for k in ('v', 'dist', 'real', 'desvio', 'estado')}
    grelhas, estradas, faixas, obstaculos = [], [], [], []
    topicos = ['/odom', f'{g}/distancia_livre', '/sim/distancia_real', '/sim/desvio_lateral',
               f'{g}/estado_manobra', f'{g}/grelha_local', '/estrada/centro', f'{g}/faixa', f'{g}/obstaculo']
    for m in read_ros2_messages(str(mcap), topics=topicos):
        t, top, msg = m.log_time_ns * 1e-9, m.channel.topic, m.ros_msg
        if top == '/odom':
            series['v'].append((t, msg.twist.twist.linear.x))
        elif top.endswith('distancia_livre'):
            series['dist'].append((t, msg.data))
        elif top == '/sim/distancia_real':
            series['real'].append((t, msg.data))
        elif top == '/sim/desvio_lateral':
            series['desvio'].append((t, msg.data))
        elif top.endswith('estado_manobra'):
            series['estado'].append((t, msg.data))
        elif top.endswith('grelha_local'):
            grelhas.append((t, msg))
        elif top == '/estrada/centro':
            estradas.append((t, msg))
        elif top.endswith('/faixa'):
            faixas.append((t, msg))
        else:
            obstaculos.append((t, msg))
    t0 = min(s[0][0] for s in series.values() if s)
    arr = {k: (np.array([a - t0 for a, _ in s]), [b for _, b in s]) for k, s in series.items()}
    rel = lambda lst: [(t - t0, msg) for t, msg in lst]
    return arr, rel(grelhas), rel(estradas), rel(faixas), rel(obstaculos)


def mais_proximo(lst, t):
    return min(lst, key=lambda e: abs(e[0] - t))[1] if lst else None


def figura_tempo(arr, saida, titulo):
    estados = ['SEM DADOS', 'PARADO', 'SEGUIR', 'CRUZEIRO']
    fig, ax = plt.subplots(4, 1, figsize=(10, 8), sharex=True,
                           gridspec_kw=dict(height_ratios=[1, 1.3, 1, 0.8], hspace=0.12))
    tv, v = arr['v']
    ax[0].plot(tv, v, color=VERDE, lw=1.8)
    ax[0].set_ylabel('velocidade (m/s)')
    td, d = arr['dist']
    alcance = max(d) if d else 37.0
    dm = np.where(np.asarray(d) >= alcance - 0.5, np.nan, d)
    ax[1].plot(td, dm, color=AZUL, lw=1.6, label='medida (faixa_livre)')
    tr, r = arr['real']
    if len(tr):
        rr = np.where(np.asarray(r) > alcance, np.nan, r)     # só enquanto o carro da frente está ao alcance
        ax[1].plot(tr, rr, color=CINZA, lw=1.0, ls='--', label='verdadeira (simulador)')
    ax[1].set_ylim(0, alcance + 2)
    ax[1].set_ylabel('distância ao\nobstáculo (m)')
    ax[1].legend(frameon=False, fontsize=9, loc='upper right')
    if len(td) and len(tv):
        vi = np.interp(td, tv, v)
        h = np.where(vi > 1.0, dm / np.maximum(vi, 1e-3), np.nan)
        ax[2].plot(td, h, color=VERMELHO, lw=1.5)
    ax[2].set_ylabel('intervalo de\ntempo (s)')
    ax[2].set_ylim(0, 5)
    te, e = arr['estado']
    ax[3].step(te, [estados.index(x) if x in estados else 0 for x in e], where='post', color='#1f2328', lw=1.4)
    ax[3].set_yticks(range(len(estados)), estados, fontsize=8.5)
    ax[3].set_xlabel('tempo (s)')
    for a in ax:
        a.grid(alpha=0.3)
        for sp in ('top', 'right'):
            a.spines[sp].set_visible(False)
    ax[0].set_title(titulo, fontsize=11)
    fig.savefig(saida, dpi=150, bbox_inches='tight')
    plt.close(fig)


def figura_instante(grelha, estrada, faixa, obst, saida, titulo):
    info = grelha.info
    w, h, r = info.width, info.height, info.resolution
    x0, y0 = info.origin.position.x, info.origin.position.y
    d = np.asarray(grelha.data, dtype=np.int16).reshape(h, w)
    img = np.where(d < 0, 1, np.where(d >= 50, 2, 0))
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.imshow(img, origin='lower', extent=[x0, x0 + w * r, y0, y0 + h * r], interpolation='nearest',
              cmap=ListedColormap(['#ffffff', '#d9dde3', '#1f2328']), vmin=0, vmax=2)
    if faixa is not None and len(faixa.polygon.points) > 2:
        ax.add_patch(Polygon([(p.x, p.y) for p in faixa.polygon.points], closed=True,
                             fc=LIMA, ec=VERDE, alpha=0.35, lw=1.2, label='faixa considerada'))
    if estrada is not None:
        ax.plot([p.pose.position.x for p in estrada.poses], [p.pose.position.y for p in estrada.poses],
                color=VERDE, lw=1.6, ls='--', label='/estrada/centro')
    if obst is not None:
        ax.plot(obst.point.x, obst.point.y, 'o', ms=9, mfc='none', mec=VERMELHO, mew=2, label='obstáculo na faixa')
    ax.add_patch(Rectangle((-0.45, -0.75), 3.45, 1.5, fc=LIMA, ec=VERDE, zorder=5))
    ax.set_xlim(x0, x0 + w * r)
    ax.set_ylim(y0, y0 + h * r)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m), base_link')
    ax.set_ylabel('y (m)')
    ax.legend(frameon=True, fontsize=8.5, loc='upper left')
    ax.set_title(titulo, fontsize=11)
    fig.savefig(saida, dpi=150, bbox_inches='tight')
    plt.close(fig)


def metricas(arr):
    te, e = arr['estado']
    if len(te) > 1:
        dur = np.diff(np.r_[te, te[-1] + np.median(np.diff(te))])
        tot = dur.sum()
        for nome in sorted(set(e)):
            print(f'  {nome:10s} {100 * dur[np.array(e) == nome].sum() / tot:5.1f} % do tempo')
    tr, r = arr['real']
    if len(tr):
        print(f'  distância mínima verdadeira ao carro da frente: {min(r):.1f} m')
    td, d = arr['dist']
    tv, v = arr['v']
    if len(td) and len(tv):
        vi = np.interp(td, tv, v)
        dd = np.asarray(d)
        ok = (vi > 2.0) & (dd < max(dd) - 0.5)
        if ok.any():
            print(f'  intervalo de tempo a seguir: mediana {np.median(dd[ok] / vi[ok]):.2f} s, '
                  f'mínimo {np.min(dd[ok] / vi[ok]):.2f} s')
    tl, lat = arr['desvio']
    if len(tl):
        print(f'  desvio lateral máximo ao centro da faixa: {np.max(np.abs(lat)):.2f} m')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path)
    ap.add_argument('--grupo', default='g00')
    ap.add_argument('--instante', type=float, default=None, help='instante (s) da figura da faixa')
    ap.add_argument('--saida', type=Path, default=Path('docs/img'))
    a = ap.parse_args()
    a.saida.mkdir(parents=True, exist_ok=True)
    arr, grelhas, estradas, faixas, obst = ler(a.bag, a.grupo)
    nome = a.bag.name
    figura_tempo(arr, a.saida / f'{nome}_manobra.png', f'Manobra: {nome}')
    t = a.instante if a.instante is not None else grelhas[len(grelhas) // 2][0]
    figura_instante(mais_proximo(grelhas, t), mais_proximo(estradas, t), mais_proximo(faixas, t),
                    mais_proximo([o for o in obst if abs(o[0] - t) < 0.3], t), a.saida / f'{nome}_faixa.png',
                    f'Instante t = {t:.1f} s')
    print(f'Figuras em {a.saida}/{nome}_manobra.png e {nome}_faixa.png')
    metricas(arr)


if __name__ == '__main__':
    main()
