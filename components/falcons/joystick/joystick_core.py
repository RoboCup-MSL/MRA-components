'''Joystick and Keyboard Input Handling'''

import sys
import pygame
import logging
import datetime
import pause
import pynput
from typing import Callable
from terminal import TerminalHandler


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
        self.key_handlers = {
            pynput.keyboard.KeyCode.from_char('p'): (lambda f: self.handle_button('A', f)),
            pynput.keyboard.KeyCode.from_char('b'): (lambda f: self.handle_button('B', f)),
            pynput.keyboard.KeyCode.from_char('g'): (lambda f: self.handle_button('X', f)),
            pynput.keyboard.KeyCode.from_char('s'): (lambda f: self.handle_button('Y', f)),
            pynput.keyboard.Key.shift: (lambda f: self.handle_button('LB', f)),
            pynput.keyboard.Key.shift_r: (lambda f: self.handle_button('RB', f)),
            pynput.keyboard.KeyCode.from_char('e'): (lambda f: self.handle_button('select', f)),
            pynput.keyboard.KeyCode.from_char('k'): (lambda f: self.handle_button('mode', f)),
            pynput.keyboard.Key.down: (lambda f: self.handle_axis('LS', 'y', f * 1.0)),
            pynput.keyboard.Key.up: (lambda f: self.handle_axis('LS', 'y', f * -1.0)),
            pynput.keyboard.Key.left: (lambda f: self.handle_axis('LS', 'x', f * -1.0)),
            pynput.keyboard.Key.right: (lambda f: self.handle_axis('LS', 'x', f * 1.0)),
            pynput.keyboard.KeyCode.from_char('5'): (lambda f: self.handle_axis('LS', 'y', f * 1.0)),
            pynput.keyboard.KeyCode.from_char('8'): (lambda f: self.handle_axis('LS', 'y', f * -1.0)),
            pynput.keyboard.KeyCode.from_char('4'): (lambda f: self.handle_axis('LS', 'x', f * -1.0)),
            pynput.keyboard.KeyCode.from_char('6'): (lambda f: self.handle_axis('LS', 'x', f * 1.0)),
            pynput.keyboard.KeyCode.from_char('7'): (lambda f: self.handle_axis('RS', 'x', f * -1.0)),
            pynput.keyboard.KeyCode.from_char('9'): (lambda f: self.handle_axis('RS', 'x', f * 1.0)),
            pynput.keyboard.KeyCode.from_char('h'): self.print_help_text,
            pynput.keyboard.KeyCode.from_char('q'): self.set_stop_flag,
        }
        self.stop_flag = False
        self.pressed_keys = set()

    def ignore_key(self, key):
        used_keys = list(self.key_handlers.keys())
        if key in used_keys:
            return False
        return True

    def handle_button(self, button, is_pressed):
        if is_pressed:
            self.buttons[button].on_press()
        else:
            self.buttons[button].on_release()

    def handle_axis(self, axis, direction, value):
        setattr(getattr(self, f'axis_{axis.lower()}'), direction, value)

    def print_help_text(self, is_pressed):
        if is_pressed:
            print("""Controls:
* movement:
    * arrow keys (or numpad 4,5,6,8): move robot forward, backward, left, right
    * numpad 7,9 keys: rotate robot
* ball handling:
    * b: toggle ballhandlers on/off
    * g: getball within a small action radius
    * left-shift+g: getball with extended action radius
    * right-shift+g: bump towards nearest teammate
* passing:
    * p: pass to the nearest teammate
    * left-shift+p: pass to the nearest obstacle
    * right-shift+p: pass to the home location
* shooting:
    * s: shoot at goal
    * left-shift+s: shoot at the home location
    * right-shift+s: bump towards goal
* lob/kicker control:
    * not mapped
* special actions:
    * e: activate keeper mode
    * shift+k: move the robot to configured home location
    * k: move the robot to the parking area

Notes:
* arrow/numpad movement cancels any running action
""")

    def set_stop_flag(self, _):
        self.stop_flag = True

    def on_event(self, key, event):
        if self.ignore_key(key):
            logging.debug('ignore key {}: {}'.format(event, key))
            return
        logging.debug('handle key {}: {}'.format(event, key))
        handler = self.key_handlers.get(key)
        if handler:
            if callable(handler):
                handler(event == 'on_press')
            else:
                handler(event)

    def on_press(self, key):
        self.pressed_keys.add(key)
        self.on_event(key, 'on_press')

    def on_release(self, key):
        self.pressed_keys.remove(key)
        self.on_event(key, 'on_release')

    def run(self):
        dt = datetime.timedelta(seconds=(1.0 / self.frequency))
        t = datetime.datetime.now()
        # prevent key echo on the terminal
        self.term = TerminalHandler()
        self.term.suppress()
        try:
            # start the listener
            # TODO: a negative side effect is that too many events are suppressed:
            # * alt-tab
            # * mouse events events
            with pynput.keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=True) as listener:
                while not self.stop_flag:
                    # detect control-C, needed because of the suppress option
                    if self.pressed_keys == {pynput.keyboard.Key.ctrl, pynput.keyboard.KeyCode.from_char('c')}:
                        break
                    # send controller state to the client
                    self.callback(self)
                    # ensure the loop runs at the given frequency
                    t += dt
                    pause.until(t)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        # restore terminal settings
        self.term.restore()
