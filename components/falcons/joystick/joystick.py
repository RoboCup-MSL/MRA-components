#!/usr/bin/env python3
"""
Joystick Controller.
"""

# python imports
import argparse
import json
import logging

# local imports
from joystick_controller import JoystickController


def button_text():
    return """
Controls (assuming xbox button layout):
* movement: use left stick to move robot forward or sideways, right stick to rotate
* toggle ballhandlers with B
* getball using X, default small action radius, use RB+X to get ball anywhere
* passing: A passes to nearest teammember, RB+A to home, LB+A to nearest obstacle
* shooting: Y shoots at goal, RB+Y to home, LB sets lob shot instead of straight shot
* keeper: select button
* kick: RT sets the power, optional LT for lob height

Notes:
* RB is right bumber, RT is right trigger, etc
* stick movement cancels any running action
    """



class Configuration:
    def __init__(self, filename):
        self.filename = filename
        self.json_data = json.load(open(filename))
    def __getattr__(self, name):
        key = name
        if key in self.json_data:
            return self.json_data[key]
        raise AttributeError(f'configuration key "{name}" not found')


def parse_args():
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=__doc__ + button_text(), formatter_class=CustomFormatter)
    parser.add_argument('-i', '--index', help='joystick index to use', type=int, default=0)
    parser.add_argument('-d', '--debug', help='enable debug logging', action='store_true')
    return parser.parse_args()


def main(args):
    # configure logging with timestamps (milliseconds)
    loglevel = (logging.DEBUG if args.debug else logging.INFO)
    logging.basicConfig(level=loglevel, format='%(asctime)s.%(msecs)03d %(levelname)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    # configuration values
    config_filename = 'joystick.json'
    joystick_config = Configuration(config_filename)
    # setup joystick
    joystick_controller = JoystickController(args.index, joystick_config)
    joystick_controller.run()


if __name__ == '__main__':
    main(parse_args())


