# -*- coding: utf-8 -*-
"""
resumo_online.py - Abre o resumo de uma sessão do online fora do Docker (no venv do offline).

  python offline/resumo_online.py data/g05_camaras_<data_hora>

O código é o do pacote (src/va_g05/va_g05/resumo_online.py), o mesmo que o online abre no fim
(no Docker: resumo_online /data/<sessão>).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "va_g05"))   # biblioteca comum (pacote va_g05)
from va_g05.resumo_online import main  # noqa: E402

if __name__ == "__main__":
    main()
