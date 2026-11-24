# -*- coding: utf-8 -*-
"""
reprodutor.py - Reprodução de um bag no Docker online (serviço "reprodutor"; processar_inter.sh --bag).

Faz de câmaras: lê as imagens do bag e publica-as nos mesmos tópicos dos drivers
(/camara<N>/image_raw, sensor_msgs/Image), ao ritmo da gravação. O processamento
(processar_tempo_real.py), o planeador e o visualizador correm sem alterações, como ao vivo: é o
que o enunciado pede ("o mesmo nó de perceção corre sem alterações com dados do bag real, com o
sistema ao vivo ou com o Gazebo").

Para ser coerente com o offline, o bag é lido com o leitura_bag.py, o mesmo do processar_bag.py:
* imagens comprimidas (bags do online: /camara<N>/image_raw/compressed) ou não (bags antigos);
* bag do online (só com os trechos): reproduz trecho a trecho, os mesmos do offline (eventos de
  /processamento/trechos), sem esperar pelos intervalos entre eles (no bag não há nada lá);
* bag sem trechos: reproduz o bag todo;
* bag sem índice (gravação interrompida) ou com blocos estragados: lido na mesma.
Os instantes das imagens (header.stamp) são os do bag: a grelha local e o aviso usam-nos, e os tempos
no terminal e no resumo são os do bag (os mesmos do offline).

Publica também /processamento/reproducao (std_msgs/String, JSON; "transient local"): o bag, o início,
a duração, os trechos e o estado ("a reproduzir", "pausa", "fim"), que o processamento e o
visualizador usam. Comando "pausa" em /processamento/comando (Espaço no visualizador): pausa / retoma.

Uso (dentro do Docker; o processar_inter.sh trata disto):
  python3 reprodutor.py <bag.mcap>            reproduz (espera que o processamento esteja pronto)
  python3 reprodutor.py <bag.mcap> --info     só mostra as câmaras e os trechos (CAMARAS=...)
Variáveis: TAXA (velocidade, 1 = tempo real; ex.: 0.5 = metade), CAMARAS_ATIVAS (só estas câmaras).
"""
import array
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import leitura_bag
import visual

TOPICO_TRECHOS = "/processamento/trechos"
TOPICO_ESTADO = "/processamento/estado_camaras"
TOPICO_COMANDO = "/processamento/comando"
TOPICO_REPRODUCAO = "/processamento/reproducao"


def analisar(bag):
    """Câmaras, tópicos, início/fim e trechos do bag (como o processar_bag.py os vê)."""
    erro = leitura_bag.verificar_mcap(bag)
    if erro:
        raise SystemExit(f"Erro: {erro}")
    indice_ok = leitura_bag.bag_tem_indice(bag)
    topicos = leitura_bag.topicos_do_bag(bag)
    cams = leitura_bag.camaras_no_bag(topicos)
    inicio, fim = leitura_bag.intervalo_do_bag(bag)
    trechos, abertos, relogio, sessao_inicio = [], {}, None, None
    if TOPICO_TRECHOS in topicos or not indice_ok:
        for m in leitura_bag.ler_mensagens(bag, [TOPICO_TRECHOS], indice_ok):
            try:
                ev = json.loads(m.ros_msg.data)
            except (ValueError, AttributeError):
                continue
            if ev.get("evento") == "sessao_inicio":
                sessao_inicio, relogio = ev.get("stamp_ns"), ev.get("relogio")
            elif ev.get("evento") == "inicio":
                abertos[ev["trecho"]] = ev["stamp_ns"]
            elif ev.get("evento") == "fim" and ev.get("trecho") in abertos:
                trechos.append({"n": ev["trecho"], "inicio_ns": abertos.pop(ev["trecho"]), "fim_ns": ev["stamp_ns"]})
    inicio = inicio or sessao_inicio
    if inicio is None or fim is None:           # sem índice: do primeiro ao último instante das imagens
        ts = [m.log_time_ns for m in leitura_bag.ler_mensagens(bag, list(cams.values()), indice_ok)]
        inicio, fim = (min(ts), max(ts)) if ts else (0, 0)
    for t in trechos:
        t["inicio_s"] = round((t["inicio_ns"] - inicio) / 1e9, 3)
        t["fim_s"] = round((t["fim_ns"] - inicio) / 1e9, 3)
    return {"bag": str(bag), "indice_ok": indice_ok, "camaras": cams, "inicio_ns": inicio, "fim_ns": fim,
            "duracao_s": round((fim - inicio) / 1e9, 3), "trechos": trechos, "relogio": relogio}


def _terminar(*_):
    raise KeyboardInterrupt


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    bag = Path(sys.argv[1])
    info = analisar(bag)
    pedidas = os.environ.get("CAMARAS_ATIVAS", "").split()
    cams = {c: t for c, t in info["camaras"].items() if not pedidas or c in pedidas}
    camaras = visual.ordenar_camaras(cams)
    if "--info" in sys.argv:
        print(f"CAMARAS={' '.join(camaras)}")
        print(f"Bag: {bag.name}  ·  {info['duracao_s']:.1f} s  ·  câmaras: {', '.join(camaras) or 'nenhuma'}"
              f"  ·  imagens: {', '.join(cams.values()) or '-'}", file=sys.stderr)
        for t in info["trechos"]:
            print(f"  trecho {t['n']}: {t['inicio_s']:.1f} s a {t['fim_s']:.1f} s", file=sys.stderr)
        if not info["trechos"]:
            print("  sem trechos: reproduz-se o bag todo", file=sys.stderr)
        sys.exit(0 if camaras else 1)
    if not camaras:
        raise SystemExit(f"Erro: o bag {bag} não tem imagens de câmaras (/camara<N>/image_raw[/compressed]).")

    signal.signal(signal.SIGTERM, _terminar)
    taxa = max(0.05, float(os.environ.get("TAXA", "1") or 1))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import Image as ROSImage
    from std_msgs.msg import String

    qos_estado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rclpy.init()
    no = Node("reprodutor")
    pubs = {c: no.create_publisher(ROSImage, f"/camara{c}/image_raw", 10) for c in camaras}
    pub_estado = no.create_publisher(String, TOPICO_REPRODUCAO, qos_estado)
    pronto = threading.Event()
    pausa = threading.Event()
    no.create_subscription(String, TOPICO_ESTADO, lambda m: pronto.set(), qos_estado)

    def comando(m):
        if m.data.strip().lower() == "pausa":
            if pausa.is_set():
                pausa.clear()
                print("Reprodução: retomada.")
            else:
                pausa.set()
                print("Reprodução: em pausa (Espaço para retomar).")
    no.create_subscription(String, TOPICO_COMANDO, comando, 10)
    threading.Thread(target=rclpy.spin, args=(no,), daemon=True).start()

    base = {k: info[k] for k in ("inicio_ns", "fim_ns", "duracao_s", "trechos", "relogio")}
    base.update(bag=bag.name, camaras=camaras, taxa=taxa)

    def estado(**k):
        m = String()
        m.data = json.dumps({**base, **k}, separators=(",", ":"), ensure_ascii=False)
        pub_estado.publish(m)

    estado(estado="à espera do processamento")
    print(f"Reprodução de {bag.name}: câmaras {', '.join(camaras)}, {info['duracao_s']:.1f} s"
          + (f", {len(info['trechos'])} trecho(s)" if info["trechos"] else ", bag todo")
          + (f", velocidade x{taxa:g}" if taxa != 1 else ""))
    if not pronto.wait(timeout=300):
        print("Aviso: o processamento não respondeu em 5 minutos; a reproduzir na mesma.")

    intervalos = ([(t["n"], t["inicio_ns"], t["fim_ns"]) for t in info["trechos"]]
                  or [(None, info["inicio_ns"], info["fim_ns"])])
    topico_cam = {t: c for c, t in cams.items()}
    try:
        for n, ini, fim in intervalos:
            if n is not None:
                print(f"Trecho {n}: {(ini - info['inicio_ns']) / 1e9:.1f} s a {(fim - info['inicio_ns']) / 1e9:.1f} s")
            t_ref, r_ref, t_estado = None, None, 0.0
            for m in leitura_bag.ler_mensagens(bag, list(topico_cam), info["indice_ok"], ini, fim):
                cam = topico_cam.get(m.channel.topic)
                if cam is None:
                    continue
                if pausa.is_set():
                    estado(estado="pausa", trecho=n, t_ns=m.log_time_ns)
                    while pausa.is_set():
                        time.sleep(0.05)
                    t_ref, t_estado = None, 0.0        # recomeça a contar o ritmo a partir daqui
                if t_ref is None:
                    t_ref, r_ref = m.log_time_ns, time.monotonic()
                espera = (m.log_time_ns - t_ref) / 1e9 / taxa - (time.monotonic() - r_ref)
                if espera > 0:
                    time.sleep(espera)
                img = publicar_imagem(pubs[cam], m.ros_msg, ROSImage)
                if img and time.monotonic() - t_estado > 0.5:
                    t_estado = time.monotonic()
                    estado(estado="a reproduzir", trecho=n, t_ns=m.log_time_ns)
        print("Reprodução: fim do bag.")
        estado(estado="fim")
        time.sleep(3)                                  # tempo para as últimas imagens serem processadas
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def publicar_imagem(pub, ros_msg, tipo):
    """Publica a imagem do bag como sensor_msgs/Image (a comprimida é descodificada para bgr8), com o
    header.stamp e o frame_id originais."""
    msg = tipo()
    msg.header.stamp.sec = int(ros_msg.header.stamp.sec)
    msg.header.stamp.nanosec = int(ros_msg.header.stamp.nanosec)
    msg.header.frame_id = ros_msg.header.frame_id
    if hasattr(ros_msg, "encoding"):                   # já sem compressão (bags antigos): tal como está
        msg.height, msg.width, msg.encoding = ros_msg.height, ros_msg.width, ros_msg.encoding
        msg.is_bigendian, msg.step = ros_msg.is_bigendian, ros_msg.step
        msg.data = array.array("B", bytes(ros_msg.data))
    else:
        frame = leitura_bag.imagem_bgr(ros_msg)
        if frame is None:
            return False
        msg.height, msg.width = frame.shape[:2]
        msg.encoding, msg.is_bigendian, msg.step = "bgr8", 0, frame.shape[1] * 3
        msg.data = array.array("B", frame.tobytes())
    pub.publish(msg)
    return True


if __name__ == "__main__":
    main()
