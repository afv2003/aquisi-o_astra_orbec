# -*- coding: utf-8 -*-
"""
aviso_faixa.py - Aviso de mudança de faixa a partir da grelha local das câmaras laterais (G5).

As câmaras do G5 fazem de retrovisores: olham para trás e para o lado e veem a faixa ao lado do carro,
do ângulo morto até algumas dezenas de metros para trás. A pergunta "posso mudar de faixa?" é, para
elas, a de um retrovisor (e dos sistemas de ângulo morto / assistente de mudança de faixa dos carros):
há algum veículo na faixa ao lado, junto ao carro, ou a aproximar-se por trás? (O planeador A* da
aula procura um caminho para a frente, numa zona que estas câmaras não veem, por isso não responde a
esta pergunta.)

ONDE ESTÁ A FAIXA AO LADO (modo_faixa no config/montagem_camaras.yaml, secção aviso_faixa):
  centro   supõe o carro no centro da sua faixa e faixas de largura_faixa: a faixa ao lado vai de
           largura/2 a 1,5 x largura para o lado, paralela ao carro;
  linhas   usa as linhas da estrada detetadas (CLRNet) e projetadas no chão (EstimadorFaixas): a linha
           da própria faixa mais perto do carro, de cada lado, é o limite interior; a seguinte (se
           for vista) o exterior, senão o interior + largura_faixa. Segue a faixa real (carro fora do
           centro, faixas mais largas ou estreitas, ligeiramente inclinadas). Sem linhas desse lado:
           SEM_INFORMACAO;
  auto     (por omissão) as linhas quando há, o centro quando não há (cada lado à parte).
As linhas são suavizadas no tempo e guardadas durante validade_linhas_s (as deteções falham imagens).

Para cada lado, olha-se para essa faixa ao longo do carro (x de x_tras a x_frente, no base_link) e dá-se
um de quatro estados:
  OCUPADA          há um veículo ou peão na zona (pegada do YOLO na grelha); diz a que distância
                   (aproximada) está o mais próximo e se está no ângulo morto (x >= x_angulo_morto);
                   ou um veículo na faixa ao lado, mesmo mais atrás (até x_tras_aproximacao), que se
                   está a aproximar e chega junto do carro em menos de ttc_mudanca_s (tempo até
                   contacto pelo crescimento da caixa: seguimento.py);
  SEM_FAIXA        a zona vista é sobretudo "não circulável" (passeio, berma): não há faixa desse lado;
  SEM_INFORMACAO   a câmara vê pouco da zona ou do ângulo morto (desligada, sem imagem recente, fora do
                   campo de visão) ou, no modo "linhas", não há linhas desse lado;
  LIVRE            vista o suficiente, circulável, sem veículos e sem nenhum a aproximar-se depressa.
"""
import numpy as np

from . import grelha_local

LADOS = ("esquerda", "direita")
SINAL = {"esquerda": 1.0, "direita": -1.0}
POR_OMISSAO = {"modo_faixa": "auto", "largura_faixa": 3.5, "x_tras": -20.0, "x_frente": 2.0,
               "x_angulo_morto": -6.0, "min_vista": 0.5, "min_vista_angulo_morto": 0.2,
               "max_nao_circulavel": 0.6, "min_celulas_veiculo": 4,
               "x_tras_aproximacao": -30.0, "ttc_mudanca_s": 4.0,
               # linhas (modo linhas / auto)
               "x_referencia_linhas": -3.0, "min_comprimento_linha": 3.0, "max_inclinacao_linha": 0.25,
               "validade_linhas_s": 1.5, "suavizacao_linhas": 0.5}


def parametros(montagem):
    p = dict(POR_OMISSAO)
    p.update((montagem or {}).get("aviso_faixa") or {})
    return p


# --------------------------------------------------------------------------- linhas -> faixas
def ajustar_linhas(linhas_chao, p):
    """Cada linha projetada no chão (pontos x, y) -> reta y = a + b x, se for comprida e quase paralela ao
    carro. Devolve [(y na referência, a, b)]."""
    res = []
    for pts in linhas_chao or []:
        pts = np.asarray(pts, float)
        if len(pts) < 3:
            continue
        sel = pts[(pts[:, 0] >= p["x_tras"] - 5) & (pts[:, 0] <= p["x_frente"])]
        if len(sel) < 3 or np.ptp(sel[:, 0]) < p["min_comprimento_linha"]:
            continue
        b, a = np.polyfit(sel[:, 0], sel[:, 1], 1)
        if abs(b) > p["max_inclinacao_linha"]:
            continue
        res.append((float(a + b * p["x_referencia_linhas"]), float(a), float(b)))
    return res


class EstimadorFaixas:
    """Limites da própria faixa e das faixas ao lado, a partir das linhas (com memória curta)."""

    def __init__(self, p):
        self.p = p
        self.guardado = {}          # (lado, "interior"/"exterior") -> (a, b, t_ns)

    def _guardar(self, chave, a, b, t_ns):
        ant = self.guardado.get(chave)
        x_ref = self.p["x_referencia_linhas"]
        if ant is not None and 0 <= (t_ns - ant[2]) / 1e9 <= self.p["validade_linhas_s"] \
                and abs((a + b * x_ref) - (ant[0] + ant[1] * x_ref)) < 1.0:
            k = float(self.p["suavizacao_linhas"])
            a, b = k * a + (1 - k) * ant[0], k * b + (1 - k) * ant[1]
        self.guardado[chave] = (a, b, t_ns)

    def _recente(self, chave, t_ns):
        g = self.guardado.get(chave)
        if g is None or abs(t_ns - g[2]) / 1e9 > self.p["validade_linhas_s"]:
            return None
        return g[0], g[1]

    def atualizar(self, linhas_chao, t_ns):
        """linhas_chao: linhas projetadas no chão (todas as câmaras, recentes). Devolve as faixas:
        {lado: {"modo", "interior": [a, b], "exterior": [a, b]}} (y = a + b x; None sem linhas no modo
        "linhas")."""
        w = float(self.p["largura_faixa"])
        retas = ajustar_linhas(linhas_chao, self.p)
        for lado in LADOS:
            s = SINAL[lado]
            do_lado = sorted((r for r in retas if s * r[0] > 0), key=lambda r: s * r[0])
            interior = next((r for r in do_lado if s * r[0] < 1.25 * w), None)
            if interior is not None:
                self._guardar((lado, "interior"), interior[1], interior[2], t_ns)
                exterior = next((r for r in do_lado if 0.6 * w <= s * (r[0] - interior[0]) <= 1.4 * w), None)
                if exterior is not None:
                    self._guardar((lado, "exterior"), exterior[1], exterior[2], t_ns)
        modo = str(self.p["modo_faixa"]).lower()
        faixas = {}
        for lado in LADOS:
            s = SINAL[lado]
            interior = self._recente((lado, "interior"), t_ns) if modo != "centro" else None
            if interior is not None:
                exterior = self._recente((lado, "exterior"), t_ns)
                if exterior is None:
                    exterior = (interior[0] + s * w, interior[1])
                faixas[lado] = {"modo": "linhas", "interior": [round(interior[0], 3), round(interior[1], 4)],
                                "exterior": [round(exterior[0], 3), round(exterior[1], 4)]}
            elif modo == "linhas":
                faixas[lado] = {"modo": "linhas", "interior": None, "exterior": None}
            else:
                faixas[lado] = {"modo": "centro", "interior": [s * w / 2, 0.0], "exterior": [s * 1.5 * w, 0.0]}
        return faixas

    def limpar(self):
        self.guardado = {}


def faixas_centro(p):
    """As faixas supondo o carro no centro (sem estado)."""
    w = float(p["largura_faixa"])
    return {lado: {"modo": "centro", "interior": [SINAL[lado] * w / 2, 0.0],
                   "exterior": [SINAL[lado] * 1.5 * w, 0.0]} for lado in LADOS}


def na_faixa(faixa, lado, x, y):
    """Máscara (ou booleano) dos pontos (x, y) entre o limite interior e o exterior da faixa ao lado."""
    if not faixa or faixa.get("interior") is None:
        return np.zeros(np.shape(x), bool)
    (ai, bi), (ae, be) = faixa["interior"], faixa["exterior"]
    yi, ye = ai + bi * np.asarray(x), ae + be * np.asarray(x)
    if lado == "esquerda":
        return (np.asarray(y) >= yi) & (np.asarray(y) <= ye)
    return (np.asarray(y) <= yi) & (np.asarray(y) >= ye)


def zonas(grelha, p, faixas=None):
    """{lado: máscara booleana da zona da faixa ao lado} na grelha."""
    faixas = faixas or faixas_centro(p)
    ao_longo = (grelha.cx >= float(p["x_tras"])) & (grelha.cx <= float(p["x_frente"]))
    return {lado: ao_longo & na_faixa(faixas.get(lado), lado, grelha.cx, grelha.cy) for lado in LADOS}


def poligono(faixa, p):
    """Os 4 cantos (x, y) da zona, para desenhar."""
    if not faixa or faixa.get("interior") is None:
        return None
    (ai, bi), (ae, be) = faixa["interior"], faixa["exterior"]
    x0, x1 = float(p["x_tras"]), float(p["x_frente"])
    return [[round(x1, 2), round(ai + bi * x1, 2)], [round(x1, 2), round(ae + be * x1, 2)],
            [round(x0, 2), round(ae + be * x0, 2)], [round(x0, 2), round(ai + bi * x0, 2)]]


# --------------------------------------------------------------------------- aviso
def avaliar(grelha, grelha_atual, veiculos, p, faixas=None, objetos=None):
    """
    Estado de cada lado. grelha: GrelhaLocal; grelha_atual: grelha combinada (int8); veiculos: células
    com veículos/peões (GrelhaLocal.veiculos); faixas: EstimadorFaixas.atualizar (None = centro);
    objetos: objetos seguidos com a posição no chão ("x", "y") e o tempo até contacto ("ttc_s").
    Devolve {lado: {estado, ...}}.
    """
    faixas = faixas or faixas_centro(p)
    res = {}
    for lado, zona in zonas(grelha, p, faixas).items():
        faixa = faixas.get(lado) or {}
        n = int(zona.sum())
        vistas = zona & (grelha_atual != grelha_local.DESCONHECIDO)
        n_vistas = int(vistas.sum())
        veic = zona & veiculos
        nao_circ = vistas & (grelha_atual == grelha_local.OCUPADO) & ~veiculos
        morto = zona & (grelha.cx >= float(p["x_angulo_morto"]))
        vista_morto = float((morto & vistas).sum() / max(int(morto.sum()), 1))
        info = {"vista": round(n_vistas / n, 2) if n else 0.0, "vista_angulo_morto": round(vista_morto, 2),
                "faixa": faixa.get("modo", "centro"), "zona": poligono(faixa, p)}
        # veículos da faixa ao lado que se aproximam (mesmo atrás da zona), pelo tempo até contacto
        perto = [o for o in (objetos or []) if o.get("x") is not None and o.get("ttc_s") is not None
                 and o.get("categoria") in ("veiculo", "bicicleta")
                 and float(p["x_tras_aproximacao"]) <= o["x"] <= float(p["x_frente"])
                 and bool(na_faixa(faixa, lado, o["x"], o["y"]))]
        aprox = min(perto, key=lambda o: o["ttc_s"]) if perto else None
        if aprox is not None:
            info["a_aproximar"] = {"ttc_s": aprox["ttc_s"], "classe": aprox["classe"], "x": aprox["x"]}
        rapido = aprox is not None and aprox["ttc_s"] <= float(p["ttc_mudanca_s"])
        ocupada = int(veic.sum()) >= int(p["min_celulas_veiculo"])
        if faixa.get("interior") is None:
            info.update(estado="SEM_INFORMACAO", motivo="sem linhas")
        elif ocupada or rapido:
            if ocupada:
                x_mais_perto = float(grelha.cx[veic].max())          # o ponto do veículo mais à frente
                info.update(distancia_m=round(x_mais_perto, 1),
                            angulo_morto=x_mais_perto >= float(p["x_angulo_morto"]))
            info.update(estado="OCUPADA", aproximacao_rapida=rapido)
        elif n == 0 or n_vistas / n < float(p["min_vista"]) or vista_morto < float(p["min_vista_angulo_morto"]):
            info.update(estado="SEM_INFORMACAO")
        elif nao_circ.sum() / max(n_vistas, 1) > float(p["max_nao_circulavel"]):
            info.update(estado="SEM_FAIXA")
        else:
            info.update(estado="LIVRE")
        res[lado] = info
    return res


def texto(info, ascii=False):
    """Texto curto de um lado (para o terminal e o painel; ascii=True para o desenho do OpenCV)."""
    e = info["estado"]
    atras = "atras" if ascii else "atrás"
    if e == "OCUPADA":
        partes = []
        if info.get("angulo_morto"):
            partes.append("angulo morto" if ascii else "no ângulo morto")
        elif info.get("distancia_m") is not None:
            partes.append(f"a {abs(info['distancia_m']):.0f} m {atras}")
        if info.get("aproximacao_rapida") and info.get("a_aproximar"):
            partes.append(f"aproxima-se: {info['a_aproximar']['ttc_s']:.1f} s")
        return "OCUPADA" + (f" ({', '.join(partes)})" if partes else "")
    if e == "SEM_INFORMACAO" and info.get("motivo"):
        return ("SEM INFORMACAO" if ascii else "SEM INFORMAÇÃO") + f" ({info['motivo']})"
    return {"SEM_FAIXA": "SEM FAIXA", "SEM_INFORMACAO": "SEM INFORMACAO" if ascii else "SEM INFORMAÇÃO"}.get(e, e)


COR = {"LIVRE": (80, 190, 80), "OCUPADA": (40, 40, 220), "SEM_FAIXA": (40, 140, 220), "SEM_INFORMACAO": (140, 140, 140)}


def desenhar(img, grelha, avisos, p):
    """Desenha a zona de cada lado na vista de cima (GrelhaLocal.imagem), com a cor do estado."""
    import cv2
    escala = img.shape[0] / grelha.largura

    def px(x, y):
        return (int(round((grelha.y_max - y) / grelha.res * escala)), int(round((grelha.x_max - x) / grelha.res * escala)))

    for lado, info in avisos.items():
        if lado not in LADOS or not isinstance(info, dict):
            continue
        cantos = info.get("zona")
        if cantos:
            q = np.array([px(x, y) for x, y in cantos], np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [q], True, COR.get(info["estado"], (200, 200, 200)), 2, cv2.LINE_AA)
    return img
