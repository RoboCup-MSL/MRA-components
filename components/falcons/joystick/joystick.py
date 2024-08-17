#!/usr/bin/env python3
"""
Joystick Controller.
"""

# python imports
import argparse
import os
import json
import logging

# local imports
from joystick_controller import JoystickController


def button_text():
    return JoystickController.button_text()



class Configuration:
    def __init__(self, config):
        if isinstance(config, dict):
            self.json_data = config
        elif isinstance(config, str):
            if not config.startswith('/'):
                config = os.path.join(os.path.dirname(__file__), config)
            if not os.path.exists(config):
                raise FileNotFoundError(f'configuration file "{config}" not found')
            self.json_data = json.load(open(config))
        else:
            raise ValueError('invalid configuration type, expected dict or str')
    def __getattr__(self, name):
        key = name
        if key in self.json_data:
            return self.json_data[key]
        raise AttributeError(f'configuration key "{name}" not found')


def arg_parser():
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        def __init__(self, prog):
            # slightly widen first column to avoid ugly line breaks
            width = 31
            argparse.ArgumentDefaultsHelpFormatter.__init__(self, prog, max_help_position=width)
            argparse.RawDescriptionHelpFormatter.__init__(self, prog, max_help_position=width)
    parser = argparse.ArgumentParser(description=__doc__ + button_text(), formatter_class=CustomFormatter)
    parser.add_argument('-i', '--index', help='joystick index to use', type=int, default=0)
    parser.add_argument('-k', '--keyboard', help='use keyboard instead of joystick', action='store_true')
    parser.add_argument('-c', '--config', help='configuration file', default='joystick.json')
    parser.add_argument('-d', '--debug', help='enable debug logging', action='store_true')
    return parser


def parse_args():
    return arg_parser().parse_args()


def main(args):
    # configure logging with timestamps (milliseconds)
    loglevel = (logging.DEBUG if args.debug else logging.INFO)
    logging.basicConfig(level=loglevel, format='%(asctime)s.%(msecs)03d %(levelname)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    # configuration values
    joystick_config = Configuration(args.config)
    # setup joystick
    joystick_controller = JoystickController('no-robot', joystick_config, args.index, args.keyboard)
    joystick_controller.run()


if __name__ == '__main__':
    main(parse_args())


