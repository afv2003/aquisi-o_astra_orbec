"""Rota gravada ("teach and repeat"): extrai de um bag a trajetória do carro e grava-a em CSV (x,y).

O CSV serve de linha de centro da faixa ao nó estrada_rota (parâmetro rota) e ao simulador.

    pip install mcap mcap-ros2-support numpy
    python tools/rota_de_bag.py data/<bag> --topico /odom --saida data/rota_<percurso>.csv
    python tools/rota_de_bag.py data/<bag> --topico /fix  --saida data/rota_<percurso>.csv

/odom (nav_msgs/Odometry): usa a posição tal como está (frame da odometria, normalmente odom).
/fix  (sensor_msgs/NavSatFix): converte latitude/longitude para metros num plano local (x para leste,
      y para norte) com origem no primeiro ponto; nesse caso o estrada_rota tem de usar um frame com
      a mesma origem (p. ex. map, publicado pelo grupo do GNSS).
Os pontos ficam espaçados de --passo metros (paragens e pontos repetidos desaparecem).
"""
import argparse
import math
from pathlib import Path

import numpy as np
from mcap_ros2.reader import read_ros2_messages

R_TERRA = 6378137.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path)
    ap.add_argument('--topico', default='/odom')
    ap.add_argument('--saida', type=Path, required=True)
    ap.add_argument('--passo', type=float, default=1.0)
    a = ap.parse_args()
    mcap = next(a.bag.glob('*.mcap')) if a.bag.is_dir() else a.bag
    pts = []
    for m in read_ros2_messages(str(mcap), topics=[a.topico]):
        msg = m.ros_msg
        if hasattr(msg, 'latitude'):
            if msg.status.status < 0 or not math.isfinite(msg.latitude):
                continue                                    # sem fix
            pts.append((msg.latitude, msg.longitude))
        else:
            pts.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    if len(pts) < 2:
        raise SystemExit(f'{a.topico}: menos de 2 mensagens válidas no bag')
    p = np.array(pts, float)
    if a.topico.endswith('fix') or abs(p[:, 0]).max() <= 90 and abs(p[:, 1]).max() <= 180 and np.ptp(p) < 1:
        lat0, lon0 = np.radians(p[0])
        lat, lon = np.radians(p[:, 0]), np.radians(p[:, 1])
        p = np.column_stack([(lon - lon0) * math.cos(lat0) * R_TERRA, (lat - lat0) * R_TERRA])
    # reamostrar com espaçamento constante ao longo do percurso
    passo = np.hypot(*np.diff(p, axis=0).T)
    s = np.r_[0.0, np.cumsum(passo)]
    novo = np.arange(0.0, s[-1], a.passo)
    x, y = np.interp(novo, s, p[:, 0]), np.interp(novo, s, p[:, 1])
    a.saida.parent.mkdir(parents=True, exist_ok=True)
    with open(a.saida, 'w') as f:
        f.write('x,y\n')
        for xi, yi in zip(x, y):
            f.write(f'{xi:.3f},{yi:.3f}\n')
    print(f'{a.saida}: {len(x)} pontos, {s[-1]:.0f} m')


if __name__ == '__main__':
    main()
