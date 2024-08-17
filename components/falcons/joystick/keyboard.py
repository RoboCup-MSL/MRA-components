# keyboard instead of joystick

# python imports
import pygame
import logging
import datetime
import pause


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
        self.prev = {key: False for key in [pygame.K_0, pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4,
                     pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9, pygame.K_g, pygame.K_b,
                     pygame.K_p, pygame.K_s, pygame.K_h, pygame.K_k, pygame.K_LSHIFT, pygame.K_RSHIFT]}
        pygame.init()
        logging.info(f'using keyboard, type h for help')

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
        # Initialize the buttons as expected by the client
        self.buttons = {name: Button() for name in ('A', 'B', 'X', 'Y', 'LB', 'RB', 'LS', 'RS', 'select', 'start', 'mode')}
        self.axis_lt = Axis1()
        self.axis_rt = Axis1()
        self.axis_lt.x = -1.0
        self.axis_rt.x = -1.0
        self.axis_ls = Axis2()
        self.axis_rs = Axis2()

    def __str__(self):
        s = f"A={int(self.buttons['A'].is_pressed)} B={int(self.buttons['B'].is_pressed)} X={int(self.buttons['X'].is_pressed)} Y={int(self.buttons['Y'].is_pressed)}"
        s += f" LB={int(self.buttons['LB'].is_pressed)} RB={int(self.buttons['RB'].is_pressed)} LS={int(self.buttons['LS'].is_pressed)} RS={int(self.buttons['RS'].is_pressed)}"
        s += f" sel={int(self.buttons['select'].is_pressed)} st={int(self.buttons['start'].is_pressed)} m={int(self.buttons['mode'].is_pressed)}"
        s += f" LT={self.axis_lt.x:.3f} RT={self.axis_rt.x:.3f}"
        s += f" LS=({self.axis_ls.x:.3f},{self.axis_ls.y:.3f}) RS=({self.axis_rs.x:.3f},{self.axis_rs.y:.3f})"
        return s

    def run(self):
        # Map keyboard keys to the original button names
        key_to_button = {
            pygame.K_g: 'A',          # G maps to A
            pygame.K_b: 'B',          # B remains B
            pygame.K_p: 'X',          # P maps to X
            pygame.K_s: 'Y',          # S maps to Y
            pygame.K_LSHIFT: 'LB',    # left-shift maps to LB
            pygame.K_RSHIFT: 'RB',    # right-shift maps to RB
            pygame.K_h: 'select',     # H maps to select
            pygame.K_k: 'mode',       # K maps to mode (for keeper)
            # Additional mappings can be added here
        }

        dt = datetime.timedelta(seconds=(1.0 / self.frequency))
        t = datetime.datetime.now()
        try:
            done = False
            while not done:
                callbacks = set()
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        done = True
                    elif event.type == pygame.KEYDOWN:
                        if event.key in key_to_button:
                            button_name = key_to_button[event.key]
                            button = self.buttons[button_name]
                            if not button.is_pressed:
                                callbacks.add(button.when_pressed)
                            button.is_pressed = True
                    elif event.type == pygame.KEYUP:
                        if event.key in key_to_button:
                            button_name = key_to_button[event.key]
                            button = self.buttons[button_name]
                            if button.is_pressed:
                                callbacks.add(button.when_released)
                            button.is_pressed = False
                for f in callbacks:
                    f()
                self.callback(self)
                t += dt
                pause.until(t)
        except KeyboardInterrupt:
            pass
