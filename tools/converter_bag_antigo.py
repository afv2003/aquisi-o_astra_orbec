# -*- coding: utf-8 -*-
"""
converter_bag_antigo.py - Converte um bag ANTIGO para o formato e o nome atuais (os do online), para
ser aceite pelo offline, pela reprodução no Docker e pelas outras ferramentas. Corre-se uma vez por
bag; os scripts do projeto só aceitam o formato atual (README, secção 3.1).

  python tools/converter_bag_antigo.py data/viagem_20261103_144234/viagem_20261103_144234_0.mcap --camara 5
  python tools/converter_bag_antigo.py data/viagem_20261110_160407/viagem_20261110_160407_0.mcap
  python tools/converter_bag_antigo.py data_realtime_inter/realtime_<data_hora>/realtime_<data_hora>_0.mcap

Bags antigos que converte:
  * Docker de aquisição antigo (data/viagem_<data_hora>/): imagens sem compressão, a viagem inteira;
    com duas câmaras em /camara5/image_raw e /camara8/image_raw, ou com uma só câmara em /image_raw
    (nesse caso, indicar qual com --camara 5 ou --camara 8);
  * sessões do online de antes desta organização (data_realtime_inter/realtime_<data_hora>/): já no
    formato atual por dentro; só mudam o nome e a pasta.

Formato atual (o que o online grava, gravacao_online.py):
  * imagens em JPEG (qualidade 95), uma câmara por tópico: /camara<N>/image_raw/compressed, com o
    instante e o frame_id originais;
  * /processamento/trechos: início da sessão, trechos e fim da sessão. Um bag antigo sem trechos fica
    com UM trecho com a viagem inteira (como se tivesse sido gravado com --registar-tudo);
  * /processamento/estado_camaras: as câmaras gravadas;
  * os outros tópicos (deteções, grelha, avisos, TF...) copiam-se como estão.
Nome atual: data/g05_camaras_<data_hora>[_<percurso>]/g05_camaras_<data_hora>[_<percurso>]_0.mcap, com a
data e a hora do nome antigo (ou da primeira imagem). O bag original não é alterado nem apagado.
"""
import argparse
import datetime
import json
import re
import sys
from pathlib import Path

from mcap.reader import make_reader, NonSeekingReader
from mcap.writer import CompressionType, Writer

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ / "src" / "va_g05"))
from va_g05 import caminhos, gravacao_online as go, leitura_bag, relogio  # noqa: E402

TOPICO_IMAGEM_ANTIGO = re.compile(r"/camara([0-9A-Za-z]+)/image_raw$")


def mensagens(caminho):
    """(canal, esquema, mensagem) por ordem de tempo; um bag sem índice lê-se sequencialmente."""
    with open(caminho, "rb") as f:
        if leitura_bag.bag_tem_indice(caminho):
            yield from make_reader(f).iter_messages(log_time_order=True)
        else:
            print("Aviso: o bag não tem índice; lido sequencialmente até onde estiver inteiro.")
            try:
                yield from NonSeekingReader(f).iter_messages()
            except Exception as e:
                print(f"Aviso: o bag acaba com dados estragados ({type(e).__name__}); converte-se o que foi lido.")


def nome_novo(bag, primeiro_ns, percurso):
    m = re.search(r"(\d{8}_\d{6})", bag.parent.name) or re.search(r"(\d{8}_\d{6})", bag.name)
    data_hora = m.group(1) if m else datetime.datetime.fromtimestamp(primeiro_ns / 1e9).strftime("%Y%m%d_%H%M%S")
    percurso = re.sub(r"[^A-Za-z0-9_-]", "_", percurso) if percurso else ""
    return f"g05_camaras_{data_hora}" + (f"_{percurso}" if percurso else "")


def main():
    ap = argparse.ArgumentParser(description="Converte um bag antigo para o formato e o nome atuais (README 3.1).")
    ap.add_argument("bag", help="o .mcap antigo")
    ap.add_argument("--camara", choices=["5", "8"], default=None,
                    help="câmara que gravou um bag antigo de UMA câmara (tópico /image_raw)")
    ap.add_argument("--percurso", default=None, help="nome do percurso, acrescentado ao nome da sessão")
    ap.add_argument("--destino", default=None, help="pasta onde criar a sessão (por omissão: data/)")
    args = ap.parse_args()

    bag = Path(args.bag)
    erro = leitura_bag.verificar_mcap(bag)
    if erro:
        sys.exit(f"Erro: {erro}")
    topicos = leitura_bag.topicos_do_bag(bag)
    if topicos and not leitura_bag.formato_atual(topicos) and bag.parent.name.startswith("g05_"):
        sys.exit("Este bag já está no formato e com o nome atuais: não é preciso converter.")
    if "/image_raw" in topicos and not args.camara:
        sys.exit("Erro: este bag é de uma só câmara (/image_raw) e o nome não diz qual. "
                 "Indique-a com --camara 5 ou --camara 8.")

    # 1.ª passagem: instantes, câmaras e se já tem trechos
    primeiro = ultimo = None
    cams, tem_trechos = set(), False
    for _, canal, msg in mensagens(bag):
        primeiro = msg.log_time if primeiro is None else primeiro
        ultimo = msg.log_time
        t = canal.topic
        if t == leitura_bag.TOPICO_TRECHOS:
            tem_trechos = True
        m = TOPICO_IMAGEM_ANTIGO.fullmatch(t)
        if m:
            cams.add(m.group(1))
        elif t == "/image_raw":
            cams.add(args.camara)
    if not cams:
        sys.exit("Erro: o bag não tem imagens de câmaras (/camara<N>/image_raw, /image_raw).")
    cams = sorted(cams)

    sessao = nome_novo(bag, primeiro, args.percurso)
    pasta = Path(args.destino) if args.destino else caminhos.DADOS
    saida = pasta / sessao / f"{sessao}_0.mcap"
    if saida.exists():
        sys.exit(f"Erro: {saida} já existe (já foi convertido?).")
    saida.parent.mkdir(parents=True, exist_ok=True)
    print(f"{bag}  ->  {saida}")
    print(f"Câmaras: {', '.join(cams)}  ·  {(ultimo - primeiro) / 1e9:.1f} s"
          + ("" if tem_trechos else "  ·  sem trechos: fica um trecho com a viagem inteira"))

    with open(saida, "wb") as f:
        w = Writer(f, compression=CompressionType.NONE)     # como o online
        w.start(profile="ros2", library="converter_bag_antigo.py")
        esq_img = w.register_schema("sensor_msgs/msg/CompressedImage", "ros2msg", go.DEFINICAO_COMPRIMIDA.encode())
        esq_txt = w.register_schema("std_msgs/msg/String", "ros2msg", go.DEFINICAO_STRING.encode())
        canal_img = {c: w.register_channel(leitura_bag.topico_imagem(c), "cdr", esq_img) for c in cams}
        canal_trechos = w.register_channel(leitura_bag.TOPICO_TRECHOS, "cdr", esq_txt)
        canal_estado = w.register_channel(go.TOPICO_ESTADO, "cdr", esq_txt)
        copiados, esquemas = {}, {}

        def texto(canal, t_ns, dados):
            w.add_message(canal, log_time=t_ns, publish_time=t_ns,
                          data=go.cdr_string(json.dumps(dados, separators=(",", ":"))))

        if not tem_trechos:
            inicio, fim = primeiro - 2, ultimo + 1
            texto(canal_trechos, inicio, {"evento": "sessao_inicio", "stamp_ns": inicio,
                                          "relogio": relogio.estado(ntp_sistema="")})
            texto(canal_estado, inicio, {"stamp_ns": inicio, "ativas": cams, "desligadas": [],
                                         "modo": "sem processamento", "pronto": True})
            texto(canal_trechos, primeiro - 1, {"evento": "inicio", "trecho": 1, "stamp_ns": primeiro - 1})
        n_img = {c: 0 for c in cams}
        for esquema, canal, msg in mensagens(bag):
            t = canal.topic
            m = TOPICO_IMAGEM_ANTIGO.fullmatch(t)
            cam = m.group(1) if m else (args.camara if t == "/image_raw" else None)
            if cam is not None:                                   # imagem sem compressão -> JPEG
                dados = go.comprimir_imagem(msg.data)
                if dados is None:
                    continue
                w.add_message(canal_img[cam], log_time=msg.log_time, publish_time=msg.log_time, data=dados)
                n_img[cam] += 1
                if sum(n_img.values()) % 300 == 0:
                    print(f"\r  {sum(n_img.values())} imagens", end="", flush=True)
                continue
            if t in (leitura_bag.TOPICO_TRECHOS, go.TOPICO_ESTADO):
                destino = canal_trechos if t == leitura_bag.TOPICO_TRECHOS else canal_estado
            elif t.endswith("/image_raw/compressed") and t.split("/")[1][len("camara"):] in canal_img:
                destino = canal_img[t.split("/")[1][len("camara"):]]
                n_img[t.split("/")[1][len("camara"):]] += 1
            else:                                                 # outros tópicos: tal como estão
                if t not in copiados:
                    if esquema.id not in esquemas:
                        esquemas[esquema.id] = w.register_schema(esquema.name, esquema.encoding, esquema.data)
                    copiados[t] = w.register_channel(t, canal.message_encoding, esquemas[esquema.id])
                destino = copiados[t]
            w.add_message(destino, log_time=msg.log_time, publish_time=msg.publish_time, data=msg.data)
        if not tem_trechos:
            texto(canal_trechos, fim, {"evento": "fim", "trecho": 1, "stamp_ns": fim,
                                       "imagens_por_camara": n_img, "deteccoes_por_camara": {c: 0 for c in cams},
                                       "intervalos_por_camara": {c: [[primeiro - 1, fim]] for c in cams}})
            texto(canal_trechos, fim + 1, {"evento": "sessao_fim", "stamp_ns": fim + 1})
        w.finish()
    print(f"\r  {sum(n_img.values())} imagens ({', '.join(f'câmara {c}: {n}' for c, n in n_img.items())})")
    print(f"Feito: {saida}  ({saida.stat().st_size / 1e6:.0f} MB; o original tinha {bag.stat().st_size / 1e6:.0f} MB)")
    print(f"Processar no offline: python offline/processar_bag.py {saida.relative_to(RAIZ) if saida.is_relative_to(RAIZ) else saida}")


if __name__ == "__main__":
    main()
