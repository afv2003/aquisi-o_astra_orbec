"""va_g05 - G5 (projeto 3, câmaras laterais): perceção das câmaras, decisão local e visualização.

Executáveis (ros2 run va_g05 <executável>; no Docker são chamados diretamente, ver compose.yaml):
  percecao       processar_tempo_real.py  imagens -> deteções, grelha local, avisos, TF, gravação
  visualizador   visualizador.py          a janela (subscreve os tópicos; não processa)
  reprodutor     reprodutor.py            reproduz um bag nos tópicos das câmaras
  resumo_online  resumo_online.py         janela do resumo de uma sessão
O offline (offline/*.py) e as ferramentas (tools/*.py) usam os mesmos módulos a partir de src/va_g05.
"""
