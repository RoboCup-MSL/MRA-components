'''Joystick and Keyboard Input Handling'''

import tty
import termios
import sys
import pygame
import logging
import datetime
import pause
import pynput
from typing import Callable


class Button:
    def __init__(self):
        self.is_pressed: bool = False
    def on_press(self):
        if not self.is_pressed:
            self.is_pressed = True
    def on_release(self):
        if self.is_pressed:
            self.is_pressed = False

class Axis1:
    def __init__(self):
        self.when_moved: Callable[[], None] = lambda: None
        self.x: float = 0.0

class Axis2:
    def __init__(self):
        self.when_moved: Callable[[], None] = lambda: None
        self.x: float = 0.0
        self.y: float = 0.0

class JoystickCore:
    def __init__(self, frequency=30.0):
        self.frequency = frequency
        self.callback: Callable[[str], None] = lambda s: None
        self.init_elements()
        self.prev = [0.0] * 6

    def init_elements(self):
        self.buttons = {name: Button() for name in ('A', 'B', 'X', 'Y', 'LB', 'RB', 'LS', 'RS', 'select', 'start', 'mode')}
        self.axis_lt = Axis1()
        self.axis_rt = Axis1()
        self.axis_ls = Axis2()
        self.axis_rs = Axis2()
        self.axis_lt.x = -1.0
        self.axis_rt.x = -1.0

    def __str__(self):
        s = f"A={int(self.buttons['A'].is_pressed)} B={int(self.buttons['B'].is_pressed)} X={int(self.buttons['X'].is_pressed)} Y={int(self.buttons['Y'].is_pressed)}"
        s += f" LB={int(self.buttons['LB'].is_pressed)} RB={int(self.buttons['RB'].is_pressed)} LS={int(self.buttons['LS'].is_pressed)} RS={int(self.buttons['RS'].is_pressed)}"
        s += f" sel={int(self.buttons['select'].is_pressed)} st={int(self.buttons['start'].is_pressed)} m={int(self.buttons['mode'].is_pressed)}"
        s += f" LT={self.axis_lt.x:.3f} RT={self.axis_rt.x:.3f}"
        s += f" LS=({self.axis_ls.x:.3f},{self.axis_ls.y:.3f}) RS=({self.axis_rs.x:.3f},{self.axis_rs.y:.3f})"
        return s

    def help_text(self):
        raise NotImplementedError("help_text should be implemented in child classes.")

class JoystickPygame(JoystickCore):
    def __init__(self, frequency=30.0, axis_threshold=0.2, joystick_index=0):
        super().__init__(frequency)
        self.axis_threshold = axis_threshold
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
                        callbacks.add(button.on_press)
                    elif event.type == pygame.JOYBUTTONUP:
                        button = buttons[event.button]
                        callbacks.add(button.on_release)
                for f in callbacks:
                    f()
                self.callback(self)
                t += dt
                pause.until(t)
        except KeyboardInterrupt:
            pass

    def help_text(self):
        print("Joystick controls help text:")

class Keyboard(JoystickCore):
    def __init__(self, frequency=30.0):
        super().__init__(frequency)
        # use pynput to detect key presses
        self.key_to_button = {
            pynput.keyboard.KeyCode.from_char('p'): 'A',
            pynput.keyboard.KeyCode.from_char('b'): 'B',
            pynput.keyboard.KeyCode.from_char('g'): 'X',
            pynput.keyboard.KeyCode.from_char('s'): 'Y',
            pynput.keyboard.Key.shift: 'LB',
            pynput.keyboard.Key.shift_r: 'RB',
            pynput.keyboard.KeyCode.from_char('e'): 'select',
            pynput.keyboard.KeyCode.from_char('k'): 'mode',
            pynput.keyboard.Key.down: 'LS_down',
            pynput.keyboard.Key.up: 'LS_up',
            pynput.keyboard.Key.left: 'LS_left',
            pynput.keyboard.Key.right: 'LS_right',
            pynput.keyboard.KeyCode.from_char('7'): 'RS_left',
            pynput.keyboard.KeyCode.from_char('9'): 'RS_right'
        }
        self.key_state = {}

    def ignore_key(self, key):
        used_keys = self.key_to_button.keys()
        if key in used_keys:
            return False
        return True

    def on_event(self, key, target_function):
        if self.ignore_key(key):
            logging.debug('ignore key {}: {}'.format(target_function, key))
            return
        logging.debug('handle key {}: {}'.format(target_function, key))
        button = self.key_to_button.get(key)
        if button:
            getattr(self.buttons[button], target_function)()
        else:
            logging.error('key not found: {}'.format(key))

    def on_press(self, key):
        self.on_event(key, 'on_press')

    def on_release(self, key):
        self.on_event(key, 'on_release')

    def run(self):
        dt = datetime.timedelta(seconds=1.0 / 30)
        t = datetime.datetime.now()
        try:
            # prevent key echo on the terminal
            orig_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)
            # start the listener
            with pynput.keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
                while True:
                    # send controller state to the client
                    self.callback(self)
                    # ensure the loop runs at the given frequency
                    t += dt
                    pause.until(t)
        except KeyboardInterrupt:
            pass
        finally:
            # restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)


    def help_text(self):
        print("Keyboard controls help text:")
        print("  p: A, b: B, g: X, s: Y")
        print("  arrow keys: LB/RB")
        print("  e: select, k: mode")
        print("  numpad 2/8: LS forward/backward")
        print("  numpad 4/6: LS left/right")
        print("  numpad 7/9: RS rotate left/right")
        print("  h: help, q: quit")
