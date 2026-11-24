# -*- coding: utf-8 -*-
"""
grelha_local.py - Grelha de ocupação local (vista de cima, à volta do carro) a partir das câmaras.

Contrato comum da aula 10 (o mesmo para todos os grupos, para o planeador A* a poder usar):
  nav_msgs/OccupancyGrid no referencial do carro (base_link), header.stamp = instante da imagem que
  lhe deu origem; células de 0,2 m; valores -1 desconhecido, 0 livre, 100 ocupado.

Como as câmaras enchem a grelha (geometria.py faz a passagem imagem <-> chão, com o chão plano):
  * Desconhecido (-1): tudo o que nenhuma câmara vê (fora do campo de visão, acima do horizonte,
    mais longe do que distancia_maxima, mais perto do que distancia_minima, onde a câmara vê sobretudo
    o próprio carro, ou de uma câmara desligada / sem imagem recente).
  * O próprio carro (secção "carro" do yaml) fica livre: é onde o planeador começa.
  * Livre (0): chão visto que o YOLOP marca como área circulável.
  * Ocupado (100): chão visto que não é área circulável (passeio, berma, obstáculos) e a "pegada"
    de cada veículo ou peão detetado pelo YOLO: a posição aproximada do objeto (o meio da base da caixa
    projetado no chão, posicao_objeto) e, a partir daí, um retângulo alinhado com a estrada, com a
    largura e o comprimento típicos do objeto. O chão
    escondido por trás do objeto (o resto da sua caixa na imagem) fica desconhecido.
  * As linhas da faixa (CLRNet) não entram na grelha (uma linha pode ser atravessada numa mudança de
    faixa); são projetadas no chão à parte (linhas_no_chao), para a vista de cima e para validação.

Com duas câmaras, cada uma tem a sua camada; a grelha publicada junta-as (ocupado ganha a livre,
livre ganha a desconhecido). Uma camada sem imagem nova há mais de validade_s deixa de contar.
"""
import numpy as np

from . import geometria

OBJETOS_OCUPAM = {"car": 4.5, "truck": 8.0, "bus": 10.0, "motorcycle": 2.0, "bicycle": 1.8, "person": 0.6}
LARGURA_OBJETO = {"car": 1.8, "truck": 2.5, "bus": 2.6, "motorcycle": 0.8, "bicycle": 0.7, "person": 0.6}
DESCONHECIDO, LIVRE, OCUPADO = -1, 0, 100


class GrelhaLocal:
    def __init__(self, montagem):
        g = montagem["grelha"]
        self.montagem = montagem
        self.res = float(g["resolucao"])
        self.x_min, self.x_max = float(g["x_min"]), float(g["x_max"])
        self.y_min, self.y_max = float(g["y_min"]), float(g["y_max"])
        self.dist_max = float(g.get("distancia_maxima", 25.0))
        self.dist_min = float(g.get("distancia_minima", 0.0))
        c = montagem.get("carro") or {}
        self.carro = (float(c.get("tras", -0.6)), float(c.get("frente", 3.0)), float(c.get("largura", 1.5)))
        self.comp_veiculo = float(g.get("comprimento_veiculo", 4.5))
        self.validade_ns = int(float(g.get("validade_s", 1.0)) * 1e9)
        self.largura = int(round((self.x_max - self.x_min) / self.res))    # células ao longo de x
        self.altura = int(round((self.y_max - self.y_min) / self.res))     # células ao longo de y
        # centros das células: linha i -> y, coluna j -> x (como no OccupancyGrid: origem em (x_min, y_min))
        jj, ii = np.meshgrid(np.arange(self.largura), np.arange(self.altura))
        self.cx = self.x_min + (jj + 0.5) * self.res
        self.cy = self.y_min + (ii + 0.5) * self.res
        self.camadas = {}          # câmara -> (grelha int8, stamp_ns)
        self.camadas_veiculos = {}  # câmara -> (células ocupadas por veículos / peões, stamp_ns)
        self._geo = {}             # (câmara, K) -> (GeometriaCamara, células vistas, u, v)

    # ------------------------------------------------------------ geometria
    def geometria(self, cam, k):
        """GeometriaCamara e as células do chão que a câmara vê (calculadas uma vez por K)."""
        chave = (str(cam), k.tobytes())
        if chave not in self._geo:
            geo = geometria.GeometriaCamara(self.montagem["camaras"][str(cam)], k)
            u, v, frente = geo.chao_para_pixel(self.cx, self.cy)
            dist = geo.distancia(self.cx, self.cy)
            vista = frente & (dist <= self.dist_max) & (dist >= self.dist_min)
            self._geo = {kk: vv for kk, vv in self._geo.items() if kk[0] != str(cam)}
            self._geo[chave] = (geo, vista, u, v)
        return self._geo[chave]

    def celulas(self, x, y):
        """(linha, coluna) da célula de cada ponto (x, y)."""
        return ((np.asarray(y) - self.y_min) / self.res, (np.asarray(x) - self.x_min) / self.res)

    # ------------------------------------------------------------ camadas
    def atualizar(self, cam, k, largura_img, altura_img, stamp_ns, mascara_estrada=None, caixas=()):
        """
        Camada de uma câmara a partir das deteções de uma imagem:
        k: intrínsecos da imagem das deteções (geometria.matriz_intrinseca); mascara_estrada: máscara
        booleana do YOLOP (ou None se não correu); caixas: [(x1, y1, x2, y2, classe, confiança)].
        """
        import cv2
        geo, vista, u, v = self.geometria(cam, k)
        dentro = vista & (u >= 0) & (u <= largura_img - 1) & (v >= 0) & (v <= altura_img - 1)
        camada = np.full((self.altura, self.largura), DESCONHECIDO, dtype=np.int8)
        if mascara_estrada is not None:
            ui = np.clip(np.round(u[dentro]).astype(int), 0, largura_img - 1)
            vi = np.clip(np.round(v[dentro]).astype(int), 0, altura_img - 1)
            camada[dentro] = np.where(mascara_estrada[vi, ui], LIVRE, OCUPADO)
        # o chão "por trás" de um objeto (dentro da sua caixa na imagem) não é visto: desconhecido
        tapado = np.zeros_like(dentro)
        for x1, y1, x2, y2, classe, _ in caixas:
            if classe in OBJETOS_OCUPAM:
                tapado |= dentro & (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
        camada[tapado] = DESCONHECIDO
        veiculos = np.zeros((self.altura, self.largura), dtype=np.uint8)
        for poligono in self.pegadas(geo, caixas, largura_img):
            ii, jj = self.celulas(poligono[:, 0], poligono[:, 1])
            pts = np.stack([jj, ii], axis=1).round().astype(np.int32)
            cv2.fillPoly(camada, [pts], OCUPADO)
            cv2.fillPoly(veiculos, [pts], 1)
        self.camadas[str(cam)] = (camada, int(stamp_ns))
        self.camadas_veiculos[str(cam)] = (veiculos.astype(bool), int(stamp_ns))
        return camada

    def posicao_objeto(self, geo, caixa, classe, largura_img=None):
        """
        Posição aproximada de um objeto no chão: (x da frente, y do centro), ou None (base acima do
        horizonte ou mais longe do que distancia_maxima). O meio da aresta de baixo da caixa é projetado
        no chão; esse ponto fica na frente do objeto mas, visto de lado, perto da sua aresta interior (a
        mais próxima da câmara): o centro fica meia largura para o lado de lá, tanto mais quanto mais de
        lado é visto (seno do ângulo entre a linha de vista e o eixo do carro).
        Uma caixa cortada pela borda esquerda ou direita da imagem não dá posição (só se vê parte do
        objeto: o meio da caixa não é o do objeto).
        """
        x1, y1, x2, y2 = caixa[:4]
        if largura_img is not None and (x1 <= 2 or x2 >= largura_img - 3):
            return None
        xs, ys, ok = geo.pixel_para_chao(np.array([(x1 + x2) / 2.0]), np.array([float(y2)]))
        if not ok[0] or geo.distancia(xs, ys)[0] > self.dist_max:
            return None
        x0, y0 = float(xs[0]), float(ys[0])
        dx, dy = x0 - geo.t[0], y0 - geo.t[1]
        seno = abs(dy) / max(np.hypot(dx, dy), 1e-6)
        larg = LARGURA_OBJETO.get(classe, 1.0)
        return x0, y0 + np.sign(dy) * larg / 2 * seno

    def pegadas(self, geo, caixas, largura_img=None):
        """
        Polígonos (x, y) no chão ocupados pelos objetos: a partir da posição aproximada (posicao_objeto),
        um retângulo alinhado com a estrada (paralelo ao carro: os veículos andam na mesma direção), com a
        largura típica da classe e o comprimento típico para o lado de lá (para longe da câmara, ao longo
        de x). Usar só o meio da base da caixa, e não os cantos, evita que um veículo visto de lado (a
        caixa inclui o comprimento dele) "transborde" para a faixa ao lado.
        """
        res = []
        for caixa in caixas:
            classe = caixa[4]
            comp = OBJETOS_OCUPAM.get(classe)
            if comp is None:
                continue
            pos = self.posicao_objeto(geo, caixa, classe, largura_img)
            if pos is None:
                continue                                   # base acima do horizonte ou muito longe
            comp = self.comp_veiculo if classe == "car" else comp
            larg = LARGURA_OBJETO.get(classe, 1.0)
            x0, yc = pos
            x_fim = x0 + (-1.0 if x0 < geo.t[0] else 1.0) * comp   # para longe da câmara, ao longo de x
            res.append(np.array([[x0, yc - larg / 2], [x0, yc + larg / 2], [x_fim, yc + larg / 2],
                                 [x_fim, yc - larg / 2]]))
        return res

    def remover(self, cam):
        self.camadas.pop(str(cam), None)
        self.camadas_veiculos.pop(str(cam), None)

    def veiculos(self, agora_ns=None):
        """Células ocupadas por veículos ou peões (as pegadas do YOLO), das camadas recentes."""
        res = np.zeros((self.altura, self.largura), dtype=bool)
        for c, t in self.camadas_veiculos.values():
            if agora_ns is None or agora_ns - t <= self.validade_ns:
                res |= c
        return res

    def combinada(self, agora_ns=None):
        """Grelha com as camadas recentes (int8, linhas = y, colunas = x)."""
        recentes = [c for c, t in self.camadas.values()
                    if agora_ns is None or agora_ns - t <= self.validade_ns]
        grelha = np.full((self.altura, self.largura), DESCONHECIDO, dtype=np.int8)
        for c in recentes:
            grelha[(c == LIVRE) & (grelha == DESCONHECIDO)] = LIVRE
        for c in recentes:
            grelha[c == OCUPADO] = OCUPADO
        tras, frente, largura = self.carro                  # onde está o próprio carro: livre
        grelha[(self.cx >= tras) & (self.cx <= frente) & (np.abs(self.cy) <= largura / 2)] = LIVRE
        return grelha

    # ------------------------------------------------------------ linhas e vista de cima
    def linhas_no_chao(self, cam, k, linhas):
        """Linhas da faixa (pontos na imagem) projetadas no chão: lista de arrays (N, 2) em metros."""
        geo = self.geometria(cam, k)[0]
        res = []
        for pts in linhas:
            pts = np.asarray(pts, float)
            if len(pts) < 2:
                continue
            x, y, ok = geo.pixel_para_chao(pts[:, 0], pts[:, 1])
            ok &= geo.distancia(np.nan_to_num(x), np.nan_to_num(y)) <= self.dist_max
            if ok.sum() >= 2:
                res.append(np.stack([x[ok], y[ok]], axis=1))
        return res

    def imagem(self, grelha, altura_px=480, linhas=None, camaras=(), titulo=None):
        """
        Vista de cima (BGR) para o ecrã: frente do carro para cima, esquerda à esquerda. Cinzento =
        desconhecido, claro = livre, vermelho escuro = ocupado; o carro a verde, a posição das câmaras
        (laranja), as linhas da faixa projetadas (amarelo) e marcas a cada 5 m (x, em metros).
        """
        import cv2
        escala = altura_px / self.largura                   # píxeis por célula
        cores = np.zeros((self.altura, self.largura, 3), np.uint8)
        cores[grelha == DESCONHECIDO] = (70, 66, 62)
        cores[grelha == LIVRE] = (225, 228, 230)
        cores[grelha == OCUPADO] = (40, 30, 160)
        # frente (x maior) para cima e esquerda (y maior) para a esquerda: rodar a matriz (linhas=y, colunas=x)
        img = np.ascontiguousarray(cores[::-1, ::-1].transpose(1, 0, 2))   # linhas = x (frente em cima), colunas = y
        img = cv2.resize(img, (int(round(self.altura * escala)), altura_px), interpolation=cv2.INTER_NEAREST)

        def px(x, y):
            return (int(round((self.y_max - y) / self.res * escala)), int(round((self.x_max - x) / self.res * escala)))

        for x in np.arange(np.ceil(self.x_min / 5) * 5, self.x_max + 1e-6, 5):   # marcas a cada 5 m
            p = px(x, self.y_max)
            cv2.line(img, (0, p[1]), (6, p[1]), (150, 150, 150), 1)
            if x % 10 == 0:
                cv2.putText(img, f"{int(x)}", (8, p[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (170, 170, 170), 1,
                            cv2.LINE_AA)
        for geo in camaras:                                  # posição de cada câmara
            c = px(geo.t[0], geo.t[1])
            cv2.circle(img, c, 3, (0, 200, 255), -1)
        for pts in linhas or []:
            q = np.array([px(x, y) for x, y in pts], np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [q], False, (0, 230, 255), 2, cv2.LINE_AA)
        tras, frente, largura = self.carro
        carro = [px(frente, largura / 2), px(tras, -largura / 2)]   # o carro (eixo traseiro na origem)
        cv2.rectangle(img, carro[0], carro[1], (60, 180, 60), -1)
        if titulo:
            cv2.putText(img, titulo, (6, altura_px - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1,
                        cv2.LINE_AA)
        return img
