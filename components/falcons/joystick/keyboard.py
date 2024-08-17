# keyboard instead of joystick

# python imports
import logging
import datetime
import pause
import sys
import termios
import tty

class Button:
    def __init__(self):
        self.when_pressed = lambda: None
        self.when_released = lambda: None
        self.is_pressed = False

class Axis1:
    def __init__(self):
        self.x = 0.0
        self.when_moved = lambda: None

class Axis2:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.when_moved = lambda: None

class KeyboardControl:
    def __init__(self, frequency=30.0):
        self.frequency = frequency
        self.callback = lambda s: None
        self.init_elements()
        logging.info('using keyboard, type h for help')

    def __str__(self):
        s = f"A={int(self.buttons['A'].is_pressed)} B={int(self.buttons['B'].is_pressed)} X={int(self.buttons['X'].is_pressed)} Y={int(self.buttons['Y'].is_pressed)}"
        s += f" LB={int(self.buttons['LB'].is_pressed)} RB={int(self.buttons['RB'].is_pressed)} LS={int(self.buttons['LS'].is_pressed)} RS={int(self.buttons['RS'].is_pressed)}"
        s += f" sel={int(self.buttons['select'].is_pressed)} st={int(self.buttons['start'].is_pressed)} m={int(self.buttons['mode'].is_pressed)}"
        s += f" LT={self.axis_lt.x:.3f} RT={self.axis_rt.x:.3f}"
        s += f" LS=({self.axis_ls.x:.3f},{self.axis_ls.y:.3f}) RS=({self.axis_rs.x:.3f},{self.axis_rs.y:.3f})"
        return s

    def print_help(self):
        help_text = """Keyboard controls:
* movement:
    * numpad 2,4,6,8 to move the robot forward, backward, left, right
    * numpad 7,9 rotate robot
* ball handling:
    * B: toggle ballhandlers on/off
    * G: getball within a small action radius
    * LS+G: getball with extended action radius
    * RS+G: bump towards nearest teammate
* passing:
    * P: pass to the nearest teammate
    * LS+P: pass to the nearest obstacle
    * RS+P: pass to the home location
* shooting:
    * S: shoot at goal
    * LS+S: shoot at the home location
    * RS+S: bump towards goal
* special actions:
    * K: activate keeper mode
    * LS+H: move the robot to configured home location
    * H: move the robot to the parking area

Notes:
* RS is right-shift, LS is left-shift
"""
        print(help_text)

    def init_elements(self):
        self.buttons = {name: Button() for name in ('A', 'B', 'X', 'Y', 'LB', 'RB', 'LS', 'RS', 'select', 'start', 'mode')}
        self.axis_lt = Axis1()
        self.axis_rt = Axis1()
        self.axis_ls = Axis2()
        self.axis_rs = Axis2()

    def run(self):
        key_to_button = {
            'g': 'A',          # G maps to A
            'b': 'B',          # B remains B
            'p': 'X',          # P maps to X
            's': 'Y',          # S maps to Y
            '\x1b[D': 'LB',    # left-shift maps to LB
            '\x1b[C': 'RB',    # right-shift maps to RB
            'h': 'select',     # H maps to select
            'k': 'mode',       # K maps to mode (for keeper)
        }

        dt = datetime.timedelta(seconds=(1.0 / self.frequency))
        t = datetime.datetime.now()
        try:
            done = False
            while not done:
                sys.stdout.flush()
                key = self.get_key()

                if key == '\x03':  # Ctrl+C to quit
                    done = True
                elif key in key_to_button:
                    button_name = key_to_button[key]
                    button = self.buttons[button_name]
                    if not button.is_pressed:
                        button.when_pressed()
                    button.is_pressed = True
                    print(f"Pressed: {button_name}")
                else:
                    for btn in self.buttons.values():
                        if btn.is_pressed:
                            btn.when_released()
                        btn.is_pressed = False

                self.callback(self)
                t += dt
                pause.until(t)
        except KeyboardInterrupt:
            pass

    def get_key(self):
        """Get a single character from standard input without blocking."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

