# -*- coding: utf-8 -*-
"""
teclado.py - Teclas carregadas no terminal, sem bloquear o programa.

Usado pelo processar_bag.py quando corre sem janela (--sem-interface): aí não há janela do OpenCV
para receber as teclas, por isso são lidas diretamente do terminal.
Funciona na Linha de Comandos/PowerShell do Windows e num terminal Linux/macOS. Não funciona se a
entrada do programa não for um terminal (por exemplo, num contentor lançado sem "-it"); nesse caso
`ativo` fica False e devem usar-se as opções --trechos ou --registar-tudo.
"""
import atexit
import os
import sys


class TecladoTerminal:
    def __init__(self):
        self.ativo = False
        self._restaurar = None
        if os.name == "nt":
            try:
                import msvcrt
                self._msvcrt = msvcrt
                self.ativo = True
            except ImportError:
                pass
        elif sys.stdin is not None and sys.stdin.isatty():
            import termios
            import tty
            fd = sys.stdin.fileno()
            antigo = termios.tcgetattr(fd)
            tty.setcbreak(fd)          # cada tecla chega logo, sem esperar pelo Enter, e não aparece no ecrã

            def restaurar():
                termios.tcsetattr(fd, termios.TCSADRAIN, antigo)
            self._restaurar = restaurar
            atexit.register(restaurar)
            self.ativo = True

    def tecla(self):
        """A tecla carregada desde a última chamada (em minúscula), ou None. Não espera."""
        if not self.ativo:
            return None
        if os.name == "nt":
            if not self._msvcrt.kbhit():
                return None
            c = self._msvcrt.getwch()
            if c in ("\x00", "\xe0"):      # teclas especiais (setas, F1...): ignoradas
                self._msvcrt.getwch()
                return None
            return c.lower()
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None

    def fechar(self):
        if self._restaurar:
            self._restaurar()
            self._restaurar = None
