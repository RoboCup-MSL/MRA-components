# core joystick interface, using pygame

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
        self.when_moved = lambda: None
        self.x = 0.0

class Axis2:
    def __init__(self):
        self.when_moved = lambda: None
        self.x = 0.0
        self.y = 0.0

class JoystickPygame:
    def __init__(self, frequency=30.0, axis_threshold=0.2, joystick_index=0):
        self.frequency = frequency
        self.axis_threshold = axis_threshold
        self.callback = lambda s: None
        self.init_elements()
        self.prev = [0.0] * 6
        pygame.init()
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        if len(joysticks) < 1:
            raise Exception('no joysticks detected by pygame')
        for idx in range(len(joysticks)):
            logging.info(f'detected joystick {idx}: {joysticks[joystick_index].get_name()}')
        logging.info(f'using joystick {joystick_index}')
        self.joystick = joysticks[joystick_index]
        self.joystick.init()

    def init_elements(self):
        self.buttons = {name: Button() for name in ('A', 'B', 'X', 'Y', 'LB', 'RB', 'LS', 'RS', 'select', 'start', 'mode')}
        self.axis_lt = Axis1()
        self.axis_rt = Axis1()
        self.axis_lt.x = -1.0
        self.axis_rt.x = -1.0
        self.axis_ls = Axis2()
        self.axis_rs = Axis2()

    def echo_onchange(self):
        for b in self.buttons.keys():
            self.buttons[b].when_pressed = lambda b=b: logging.info(f'button {b} pressed')
            self.buttons[b].when_released = lambda b=b: logging.info(f'button {b} released')
        self.axis_lt.when_moved = lambda: logging.info(f'trigger LT got value {self.axis_lt.x:.3f}')
        self.axis_rt.when_moved = lambda: logging.info(f'trigger RT got value {self.axis_rt.x:.3f}')
        self.axis_ls.when_moved = lambda: logging.info(f'stick LS got value ({self.axis_ls.x:.3f},{self.axis_ls.y:.3f})')
        self.axis_rs.when_moved = lambda: logging.info(f'stick RS got value ({self.axis_rs.x:.3f},{self.axis_rs.y:.3f})')

    def __str__(self):
        s = f"A={int(self.buttons['A'].is_pressed)} B={int(self.buttons['B'].is_pressed)} X={int(self.buttons['X'].is_pressed)} Y={int(self.buttons['Y'].is_pressed)}"
        s += f" LB={int(self.buttons['LB'].is_pressed)} RB={int(self.buttons['RB'].is_pressed)} LS={int(self.buttons['LS'].is_pressed)} RS={int(self.buttons['RS'].is_pressed)}"
        s += f" sel={int(self.buttons['select'].is_pressed)} st={int(self.buttons['start'].is_pressed)} m={int(self.buttons['mode'].is_pressed)}"
        s += f" LT={self.axis_lt.x:.3f} RT={self.axis_rt.x:.3f}"
        s += f" LS=({self.axis_ls.x:.3f},{self.axis_ls.y:.3f}) RS=({self.axis_rs.x:.3f},{self.axis_rs.y:.3f})"
        return s

    def run(self):
        dt = datetime.timedelta(seconds=(1.0 / self.frequency))
        t = datetime.datetime.now()
        try:
            buttons = {i: self.buttons[name] for i, name in enumerate(self.buttons)}
            done = False
            while not done:
                callbacks = set()
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        done = True
                    if event.type == pygame.JOYAXISMOTION:
                        ax = None
                        if event.axis == 0:
                            self.axis_ls.x = event.value
                            ax = self.axis_ls
                        elif event.axis == 1:
                            self.axis_ls.y = event.value
                            ax = self.axis_ls
                        elif event.axis == 2:
                            self.axis_lt.x = event.value
                            ax = self.axis_lt
                        elif event.axis == 3:
                            self.axis_rs.x = event.value
                            ax = self.axis_rs
                        elif event.axis == 4:
                            self.axis_rs.y = event.value
                            ax = self.axis_rs
                        elif event.axis == 5:
                            self.axis_rt.x = event.value
                            ax = self.axis_rt
                        delta = abs(self.prev[event.axis] - event.value)
                        if delta > self.axis_threshold:
                            callbacks.add(ax.when_moved)
                            self.prev[event.axis] = event.value
                    elif event.type == pygame.JOYBUTTONDOWN:
                        button = buttons[event.button]
                        if not button.is_pressed:
                            callbacks.add(button.when_pressed)
                        button.is_pressed = True
                    elif event.type == pygame.JOYBUTTONUP:
                        button = buttons[event.button]
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
