import sys
import termios
import tty


class TerminalHandler:
    def __init__(self):
        self.orig_settings = termios.tcgetattr(sys.stdin)
    def suppress(self):
        tty.setcbreak(sys.stdin)
    def restore(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
