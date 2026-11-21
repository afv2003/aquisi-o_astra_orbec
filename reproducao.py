# -*- coding: utf-8 -*-
"""
reproducao.py - Junta as imagens do bag original com as deteções de um registo.

Usado pelo reproduzir_registo.py (vídeo no ecrã ou em .mp4) e pelo gui_resultados.py (vídeo na
GUI), para os dois mostrarem exatamente o mesmo: as imagens lidas e corrigidas como no
processar_bag.py (leitura_bag.py) e as deteções registadas desenhadas com o visual.py.
Os modelos não voltam a correr.

Com um registo de duas câmaras, as imagens são juntadas lado a lado (5 à esquerda, 8 à direita),
como no processar_bag.py; pode também escolher-se só uma das câmaras (camaras=["5"]).
Num registo juntado (juntar_registos.py), cada trecho diz em que intervalos cada câmara foi
registada ("intervalos_por_camara"); fora deles, o lado dessa câmara aparece escuro, com uma mensagem.
O mesmo acontece num registo feito a partir de um bag do online (gravado só nos trechos): os trechos
são os do online ("origem": "online") e, se uma câmara foi desligada no online (tecla D) durante parte
de um trecho, o vídeo dela só existe nos intervalos em que esteve ligada.
"""
from pathlib import Path

import numpy as np

import leitura_bag
import registo
import visual

MODELOS = ("objetos", "linhas", "estrada")


class ErroRegisto(Exception):
    """Registo ou bag original em falta ou inválido (a mensagem explica o que fazer)."""


class Reproducao:
    def __init__(self, pasta_ensaio, bag=None, camaras=None):
        self.pasta = registo.pasta_do_ensaio(pasta_ensaio)
        self.ensaio = registo.ler_ensaio(self.pasta)
        if self.ensaio is None:
            raise ErroRegisto(f"'{self.pasta}' não tem ensaio.json (não é uma pasta de registo do processar_bag.py).")
        self.trechos = self.ensaio.get("trechos", [])
        # registo feito a partir de um bag do online (gravado só nos trechos)
        self.online = (self.ensaio.get("origem") or {}).get("tipo") == "online"
        if not self.trechos:
            raise ErroRegisto("O registo não tem trechos.")

        # Câmaras do registo e as escolhidas (por omissão, todas), pela ordem do ecrã
        self.info_camaras = {c["camara"]: c for c in registo.camaras_do_ensaio(self.ensaio)}
        self.camaras_registo = list(self.info_camaras)
        if camaras:
            camaras = [str(c) for c in camaras]
            fora = [c for c in camaras if c not in self.info_camaras]
            if fora:
                raise ErroRegisto(f"O registo não tem a(s) câmara(s) {', '.join(fora)} "
                                  f"(tem: {registo.texto_camaras(self.camaras_registo)}).")
            self.camaras = visual.ordenar_camaras(camaras)
        else:
            self.camaras = list(self.camaras_registo)

        todas = registo.ler_deteccoes(self.pasta / self.ensaio.get("ficheiro_deteccoes", "deteccoes.mcap"))
        self.deteccoes = {c: todas.get(c, {}) for c in self.camaras}      # {câmara: {instante_ns: dados}}

        # Bag original: onde estava no processamento, ou em data/<viagem>/ (ou o indicado)
        info_bag = self.ensaio["bag"]
        candidatos = [Path(bag)] if bag else [
            Path(info_bag["ficheiro"]),
            Path("data") / Path(info_bag["ficheiro"]).parent.name / info_bag["nome"],
        ]
        self.bag = next((c for c in candidatos if c.exists()), None)
        if self.bag is None:
            raise ErroRegisto("Não encontrei o bag original. Procurei em:\n  "
                              + "\n  ".join(str(c) for c in candidatos)
                              + "\nIndique onde está (no reproduzir_registo.py, com --bag <caminho do .mcap>).")
        erro = leitura_bag.verificar_mcap(self.bag)
        if erro:
            raise ErroRegisto(erro)
        tamanho = info_bag.get("tamanho_bytes")
        self.aviso_tamanho = (tamanho is not None and self.bag.stat().st_size != tamanho)
        self.indice_ok = leitura_bag.bag_tem_indice(self.bag)
        self.topico = {}                                                   # câmara -> tópico da imagem
        for c in self.camaras:
            t = self.info_camaras[c].get("topico_imagem") or info_bag.get("topico_imagem")
            if not t:
                raise ErroRegisto(f"O registo não diz em que tópico do bag estão as imagens da câmara {c}.")
            self.topico[c] = t
        self.camara_do_topico = {t: c for c, t in self.topico.items()}
        self.inicio_bag_ns = info_bag.get("inicio_ns") or min(t["inicio_ns"] for t in self.trechos)
        self.duracao_bag = info_bag.get("duracao_s") or 0.0

        self.corretor = {}
        for c in self.camaras:
            calib = self.info_camaras[c].get("calibracao")
            if calib:
                self.corretor[c] = leitura_bag.CorretorDistorcao(leitura_bag.calibracao_de_dict(calib))
        modelos = self.ensaio.get("modelos", {})
        self.nome_modelo = {k: modelos.get(k, {}).get("nome", k) for k in MODELOS}
        self.carregado = {k: bool(modelos.get(k, {}).get("carregado", True)) for k in MODELOS}

    @property
    def duas(self):
        return len(self.camaras) > 1

    def trecho(self, n):
        return next(t for t in self.trechos if t["n"] == n)

    def imagens_por_camara(self, trecho):
        """{câmara: número de imagens no trecho} das câmaras escolhidas."""
        por_cam = trecho.get("imagens_por_camara")
        if not por_cam:                                    # registos antigos: uma câmara
            return {self.camaras[0]: trecho["imagens"]}
        return {c: por_cam.get(str(c), 0) for c in self.camaras}

    def camara_principal(self, trecho):
        """A câmara que marca o ritmo do vídeo (a que tem mais imagens no trecho)."""
        por_cam = self.imagens_por_camara(trecho)
        return max(self.camaras, key=lambda c: por_cam.get(c, 0))

    def intervalos(self, trecho, cam):
        """Intervalos [(início_ns, fim_ns), ...] em que a câmara foi registada no trecho (todo o trecho
        num registo normal; num registo juntado, os trechos originais dessa câmara)."""
        por_cam = trecho.get("intervalos_por_camara")
        if not por_cam:
            return [(trecho["inicio_ns"], trecho["fim_ns"])]
        return [tuple(i) for i in por_cam.get(str(cam), [])]

    def registada(self, trecho, cam, t_ns):
        return any(a <= t_ns <= b for a, b in self.intervalos(trecho, cam))

    def fps(self, trecho):
        """Ritmo da gravação no trecho (imagens por segundo da câmara principal)."""
        cam = self.camara_principal(trecho)
        n = self.imagens_por_camara(trecho).get(cam, 0) or trecho["imagens"]
        dur = sum(b - a for a, b in self.intervalos(trecho, cam)) / 1e9
        return (n - 1) / dur if dur > 0 and n > 1 else 30.0

    def n_imagens(self, trecho):
        """Número de imagens do vídeo do trecho (as da câmara principal; num registo juntado, uma
        estimativa: o vídeo cobre o trecho todo, incluindo onde essa câmara não foi registada)."""
        if not trecho.get("intervalos_por_camara") or self.online:
            # (num bag do online, a câmara principal é a que esteve sempre ligada: tem as imagens todas)
            return self.imagens_por_camara(trecho).get(self.camara_principal(trecho), 0) or trecho["imagens"]
        return int(round(self.fps(trecho) * (trecho["fim_ns"] - trecho["inicio_ns"]) / 1e9)) + 1

    def imagens(self, trecho, desenhar=True):
        """
        Imagens de um trecho, já com as deteções desenhadas e o painel, como no processar_bag.py
        (ou, com desenhar=False, só corrigidas da distorção, sem desenhos).
        Gera (instante_ns, imagem BGR, {câmara: dados das deteções ou None}); no dicionário só entram
        as câmaras que já têm imagem e que foram registadas nesse instante (num registo juntado, uma
        câmara fora dos seus intervalos aparece escura, com uma mensagem).
        Com duas câmaras, a imagem tem as câmaras lado a lado; sai uma imagem por cada imagem da
        câmara principal, com a imagem mais recente da outra câmara.
        """
        principal = self.camara_principal(trecho)
        ultima = {c: None for c in self.camaras}
        dados_cam = {}
        for msg in leitura_bag.ler_mensagens(self.bag, list(self.topico.values()), self.indice_ok,
                                             trecho["inicio_ns"], trecho["fim_ns"]):
            cam = self.camara_do_topico.get(msg.channel.topic)
            if cam is None and msg.channel.topic not in self.camara_do_topico:
                continue
            frame = leitura_bag.imagem_bgr(msg.ros_msg)
            if frame is None:
                continue
            if not self.registada(trecho, cam, msg.log_time_ns):
                # câmara não registada neste intervalo (registo juntado): escuro, com uma mensagem
                h, w = frame.shape[:2]
                ultima[cam] = self._sem_registo(trecho, cam, msg.log_time_ns, w, h)
                dados_cam.pop(cam, None)
                if cam == principal:
                    yield msg.log_time_ns, self.juntar(ultima, trecho, msg.log_time_ns), dict(dados_cam)
                continue
            dados = self.deteccoes[cam].get(msg.log_time_ns)
            dados_cam[cam] = dados
            if desenhar:
                ultima[cam] = self.desenhar(frame, dados, trecho, msg.log_time_ns, cam)
            else:
                ultima[cam] = self.corrigir(frame, dados, cam)
            if cam != principal:
                continue
            yield msg.log_time_ns, self.juntar(ultima, trecho, msg.log_time_ns), dict(dados_cam)

    def _sem_registo(self, trecho, cam, t_ns, w, h):
        """Lado escuro de uma câmara que não foi registada neste instante (registo juntado, ou câmara
        desligada no online)."""
        if self.online:
            return visual.imagem_espera(w, h, f"Câmara {cam}: desligada no online",
                                        "tecla D no online: sem vídeo nem deteções desta câmara aqui")
        outras = [c for c in self.camaras if c != cam and self.registada(trecho, c, t_ns)]
        return visual.imagem_espera(w, h, f"Câmara {cam}: sem registo neste intervalo",
                                    (f"só a câmara {registo.texto_camaras(outras)} foi registada aqui" if outras
                                     else "nenhuma câmara foi registada aqui"))

    def juntar(self, ultima, trecho=None, t_ns=None):
        """Imagens das câmaras lado a lado (uma câmara ainda sem imagem aparece a escuro)."""
        ref = next(im for im in ultima.values() if im is not None)
        h, w = ref.shape[:2]
        partes = []
        for c in self.camaras:
            # fora dos intervalos registados, a câmara aparece escura (mesmo que não chegue nenhuma
            # imagem dela nesse intervalo, como no bag do online com a câmara desligada)
            if trecho is not None and t_ns is not None and not self.registada(trecho, c, t_ns):
                partes.append(self._sem_registo(trecho, c, t_ns, w, h))
            elif ultima[c] is not None:
                partes.append(ultima[c])
            else:
                partes.append(visual.imagem_espera(w, h, f"Câmara {c}: sem imagem ainda"))
        return visual.lado_a_lado(partes)

    def _corrigida(self, dados, cam):
        por_omissao = cam in self.corretor
        return (bool(dados.get("corrigida", por_omissao)) if dados else por_omissao) and cam in self.corretor

    def corrigir(self, frame, dados, cam=None):
        """Correção da distorção, como estava no processamento dessa imagem."""
        cam = self.camaras[0] if cam is None else cam
        return self.corretor[cam](frame) if self._corrigida(dados, cam) else frame

    def desenhar(self, frame, dados, trecho, t_ns, cam=None):
        cam = self.camaras[0] if cam is None else cam
        corrigida = self._corrigida(dados, cam)
        if corrigida:
            frame = self.corretor[cam](frame)
        h, w = frame.shape[:2]

        caixas, linhas_pts, mascara = [], [], None
        ativos = dict(self.carregado)
        if dados:
            ativos = {k: bool(v) for k, v in (dados.get("modelos_ativos") or self.carregado).items()}
            caixas = [(*o["caixa"], o["classe"], o["confianca"]) for o in dados.get("objetos", [])]
            linhas_pts = [np.array(l, dtype=np.float32) for l in dados.get("linhas", []) if len(l) > 1]
            if ativos.get("estrada") and dados.get("estrada_contornos") is not None:
                mascara = registo.mascara_dos_contornos(dados["estrada_contornos"], dados.get("estrada_buracos", []), w, h)
        visual.desenhar_estrada(frame, mascara)
        visual.desenhar_linhas(frame, linhas_pts)
        visual.desenhar_caixas(frame, caixas)

        n_obj, n_lin = len(caixas), len(linhas_pts)
        pct = dados.get("estrada_percentagem", 0.0) if dados else 0.0
        valores = {"objetos": f"{n_obj} objeto" + ("" if n_obj == 1 else "s"),
                   "linhas": f"{n_lin} linha" + ("" if n_lin == 1 else "s"),
                   "estrada": f"{pct:.0%} da imagem"}
        texto_cam = f"câmara {cam}" if cam is not None else "câmara não indicada"
        info = {"tempo": (t_ns - self.inicio_bag_ns) / 1e9, "total": self.duracao_bag,
                "subtitulo": (f"Registo  ·  {texto_cam}  ·  trecho {trecho['n']}"
                              + ("" if dados else "  ·  sem deteções")),
                "cor_subtitulo": "#9CA3AF" if dados else "#F59E0B",
                "modelos": [(k, self.nome_modelo[k], valores[k], None, ativos.get(k, False), self.carregado[k])
                            for k in MODELOS],
                "rodape": f"{self.pasta.name}  ·  " + ("corrigida" if corrigida else "original")}
        return visual.compor(frame, caixas, info)
