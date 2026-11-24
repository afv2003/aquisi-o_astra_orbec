# -*- coding: utf-8 -*-
"""
relogio.py - Verificação da hora do PC no arranque do online (processar_tempo_real.py).

O enunciado: "A gravação é feita com o relógio do portátil (...) o portátil deve ter a hora certa
(NTP) para os bags serem comparáveis mais tarde." Os instantes das imagens (header.stamp) são postos
pelo driver da câmara com o relógio do PC; se o relógio estiver adiantado ou atrasado, os bags deste
PC não coincidem com os de outros grupos (parte 2).

Duas verificações, as duas sem instalar nada no Ubuntu:
1. O processar_inter.sh pergunta ao sistema (timedatectl, do systemd, que já vem no Ubuntu) se o
   relógio está sincronizado por NTP, e passa a resposta ao contentor (RELOGIO_NTP=yes/no).
2. Aqui, dentro do contentor, mede-se o desvio do relógio face a um servidor NTP (SNTP, um pedido
   UDP à porta 123; servidor em SERVIDOR_NTP, por omissão pool.ntp.org). Sem rede (no carro, por
   exemplo) a medição falha e fica só a resposta do sistema.

O resultado é mostrado no terminal, gravado no bag (evento "sessao_inicio" de /processamento/trechos)
e no resumo_online.json, e o offline copia-o para o registo (ensaio.json).
"""
import os
import socket
import struct
import time

SERVIDOR = os.environ.get("SERVIDOR_NTP", "pool.ntp.org")
LIMITE_MS = 100.0          # desvio aceitável (as câmaras dão uma imagem a cada ~33 ms)
_EPOCA_NTP = 2208988800    # segundos de 1900-01-01 (NTP) a 1970-01-01 (Unix)


def _ntp_para_ns(seg, frac):
    return (seg - _EPOCA_NTP) * 1_000_000_000 + (frac * 1_000_000_000 >> 32)


def medir(servidor=SERVIDOR, timeout=1.5):
    """Desvio do relógio deste PC face ao servidor NTP. Devolve {servidor, desvio_ms, atraso_ms} ou None."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(timeout)
            pedido = bytearray(48)
            pedido[0] = 0x23                         # LI = 0, versão 4, modo 3 (cliente)
            t0 = time.time_ns()
            s.sendto(bytes(pedido), (servidor, 123))
            resposta, _ = s.recvfrom(48)
            t3 = time.time_ns()
    except (OSError, socket.timeout):
        return None
    if len(resposta) < 48 or (resposta[0] & 0x07) != 4:
        return None
    t1 = _ntp_para_ns(*struct.unpack("!II", resposta[32:40]))   # receção no servidor
    t2 = _ntp_para_ns(*struct.unpack("!II", resposta[40:48]))   # envio do servidor
    desvio = ((t1 - t0) + (t2 - t3)) / 2                       # servidor - este PC
    atraso = (t3 - t0) - (t2 - t1)
    return {"servidor": servidor, "desvio_ms": round(desvio / 1e6, 1), "atraso_ms": round(atraso / 1e6, 1)}


def estado(ntp_sistema=None, medicao=None):
    """
    Junta a resposta do sistema (RELOGIO_NTP: "yes"/"no"/vazio) e a medição. Devolve um dicionário
    com "ok" (True/False/None = não se sabe), "texto" (para o terminal) e os dados.
    """
    if ntp_sistema is None:
        ntp_sistema = os.environ.get("RELOGIO_NTP", "")
    sistema = {"yes": True, "no": False}.get(ntp_sistema.strip().lower())
    res = {"sincronizado_sistema": sistema, "medicao": medicao, "limite_ms": LIMITE_MS,
           "verificado_em_ns": time.time_ns()}
    if medicao is not None:
        d = medicao["desvio_ms"]
        res["ok"] = abs(d) <= LIMITE_MS
        quanto = f"{d:+.0f} ms" if abs(d) < 1000 else f"{d / 1000:+.1f} s"
        res["texto"] = (f"desvio de {quanto} face a {medicao['servidor']}"
                        + ("" if sistema is None else
                           ("; o sistema diz que está sincronizado" if sistema else "; o NTP do sistema está desligado")))
    elif sistema is not None:
        res["ok"] = sistema
        res["texto"] = "segundo o sistema; sem rede para medir o desvio"
    else:
        res["ok"] = None
        res["texto"] = "sem rede para medir e sem a resposta do sistema"
    return res


def verificar(timeout=1.5):
    """Mede e devolve o estado (chamado no arranque do online)."""
    return estado(medicao=medir(timeout=timeout))


def _estado_txt(est):
    return {True: "sincronizado", False: "NÃO sincronizado", None: "não verificado"}[est.get("ok")]


def texto_completo(est):
    """Linha(s) para o terminal, com o que fazer quando o relógio não está certo."""
    if est.get("ok"):
        return f"Relógio do PC: sincronizado ({est['texto']})"
    if est.get("ok") is None:
        return (f"Relógio do PC: não verificado ({est['texto']}). Antes de sair, com internet, confirme com: "
                "timedatectl (\"System clock synchronized: yes\").")
    return (f"ATENÇÃO - relógio do PC NÃO sincronizado ({est['texto']}). Os instantes das imagens vêm deste "
            "relógio: os bags não ficam comparáveis com os de outros PCs. Com internet: sudo timedatectl "
            "set-ntp true, espere um pouco e confirme com: timedatectl (README, secção 4.12).")


def resumo_curto(est):
    """Texto curto (resumo e GUI)."""
    if not est:
        return "não verificado"
    return f"{_estado_txt(est)} ({est.get('texto', '')})"
