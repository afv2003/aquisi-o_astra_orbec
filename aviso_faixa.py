# -*- coding: utf-8 -*-
"""
aviso_faixa.py - Aviso de mudança de faixa a partir da grelha local das câmaras laterais (G5).

As câmaras do G5 olham para trás e para o lado (como retrovisores): veem a faixa ao lado do carro,
do ângulo morto até algumas dezenas de metros para trás. A pergunta "posso mudar de faixa?" é,
para estas câmaras, a dos sistemas de ângulo morto dos carros: há algum veículo na faixa ao lado,
junto ao carro ou a aproximar-se por trás? (O planeador A* da aula procura um caminho para a
frente, numa zona que estas câmaras não veem, por isso não responde a esta pergunta.)

Para cada lado, olha-se para uma zona da grelha (no base_link):
  faixa ao lado:  y de largura/2 a 1,5 x largura (esquerda; o simétrico à direita), supondo o carro
                  no centro da sua faixa;
  ao longo do carro:  x de x_tras (ex. -20 m) a x_frente (ex. 2 m).
e dá um de quatro estados:
  OCUPADA          há um veículo ou peão (pegada do YOLO) na zona; diz a que distância está o mais
                   próximo (x, em m: negativo = atrás) e se está no ângulo morto (x >= x_angulo_morto);
  SEM_FAIXA        a zona vista é sobretudo "não circulável" (passeio, berma): não há faixa desse lado;
  SEM_INFORMACAO   a câmara vê pouco da zona, ou do ângulo morto (desligada, sem imagem recente, ou fora
                   do campo de visão);
  LIVRE            vista o suficiente, circulável e sem veículos.
"""
import grelha_local

LADOS = ("esquerda", "direita")
POR_OMISSAO = {"largura_faixa": 3.5, "x_tras": -20.0, "x_frente": 2.0, "x_angulo_morto": -6.0,
               "min_vista": 0.5, "min_vista_angulo_morto": 0.2, "max_nao_circulavel": 0.6,
               "min_celulas_veiculo": 4}


def parametros(montagem):
    p = dict(POR_OMISSAO)
    p.update((montagem or {}).get("aviso_faixa") or {})
    return p


def zonas(grelha, p):
    """{lado: máscara booleana da zona da faixa ao lado} na grelha."""
    w = float(p["largura_faixa"])
    ao_longo = (grelha.cx >= float(p["x_tras"])) & (grelha.cx <= float(p["x_frente"]))
    return {"esquerda": ao_longo & (grelha.cy >= w / 2) & (grelha.cy <= 1.5 * w),
            "direita": ao_longo & (grelha.cy <= -w / 2) & (grelha.cy >= -1.5 * w)}


def avaliar(grelha, grelha_atual, veiculos, p):
    """
    Estado de cada lado. grelha: GrelhaLocal; grelha_atual: grelha combinada (int8); veiculos:
    células com veículos/peões (GrelhaLocal.veiculos). Devolve {lado: {estado, ...}}.
    """
    res = {}
    for lado, zona in zonas(grelha, p).items():
        n = int(zona.sum())
        vistas = zona & (grelha_atual != grelha_local.DESCONHECIDO)
        n_vistas = int(vistas.sum())
        veic = zona & veiculos
        nao_circ = vistas & (grelha_atual == grelha_local.OCUPADO) & ~veiculos
        morto = zona & (grelha.cx >= float(p["x_angulo_morto"]))
        vista_morto = float((morto & vistas).sum() / max(int(morto.sum()), 1))
        info = {"vista": round(n_vistas / n, 2) if n else 0.0, "vista_angulo_morto": round(vista_morto, 2)}
        if int(veic.sum()) >= int(p["min_celulas_veiculo"]):
            x_mais_perto = float(grelha.cx[veic].max())          # o ponto do veículo mais à frente
            info.update(estado="OCUPADA", distancia_m=round(x_mais_perto, 1),
                        angulo_morto=x_mais_perto >= float(p["x_angulo_morto"]))
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
    if e == "OCUPADA":
        if info.get("angulo_morto"):
            onde = "angulo morto" if ascii else "no ângulo morto"
        else:
            onde = f"a {abs(info['distancia_m']):.0f} m " + ("atras" if ascii else "atrás")
        return f"OCUPADA ({onde})"
    return {"SEM_FAIXA": "SEM FAIXA", "SEM_INFORMACAO": "SEM INFORMACAO" if ascii else "SEM INFORMAÇÃO"}.get(e, e)


COR = {"LIVRE": (80, 190, 80), "OCUPADA": (40, 40, 220), "SEM_FAIXA": (40, 140, 220), "SEM_INFORMACAO": (140, 140, 140)}


def desenhar(img, grelha, avisos, p):
    """Desenha as zonas e o estado de cada lado sobre a vista de cima (GrelhaLocal.imagem)."""
    import cv2
    escala = img.shape[0] / grelha.largura
    w = float(p["largura_faixa"])
    for lado, info in avisos.items():
        y0, y1 = (w / 2, 1.5 * w) if lado == "esquerda" else (-1.5 * w, -w / 2)
        x0, x1 = float(p["x_tras"]), float(p["x_frente"])
        a = (int(round((grelha.y_max - y1) / grelha.res * escala)), int(round((grelha.x_max - x1) / grelha.res * escala)))
        b = (int(round((grelha.y_max - y0) / grelha.res * escala)), int(round((grelha.x_max - x0) / grelha.res * escala)))
        cv2.rectangle(img, a, b, COR[info["estado"]], 2)
    linha = 16
    for lado in LADOS:
        if lado in avisos:
            cv2.putText(img, f"{'E' if lado == 'esquerda' else 'D'}: {texto(avisos[lado], ascii=True)}", (6, linha),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, COR[avisos[lado]["estado"]], 1, cv2.LINE_AA)
            linha += 16
    return img
