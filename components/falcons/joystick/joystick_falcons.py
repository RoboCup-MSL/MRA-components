#!/usr/bin/env python3
"""
Joystick Controller for Falcons Robot.

Uses generic joystick.py and instantiates the robot interface for Falcons software (using RTDB).

This script controls a robot using a joystick / gamepad.
It makes use of ActionPlanning.

It needs to be called via bazel run, so that the pybind module can be found.
"""

# python imports
import os
import json
import logging
from google.protobuf import json_format

# local imports
from components.falcons.joystick import joystick
import falcons_interface
from terminal import TerminalHandler


def parse_args():
    parser = joystick.arg_parser()
    # customize
    parser.description = __doc__ + joystick.button_text()
    parser.set_defaults(config=os.path.join(os.getenv('FALCONS_CODE_PATH'), 'config', 'joystick.json'))
    parser.add_argument('-t', '--testmode', help='test mode, no RTDB / robot connection, just test the buttons', action='store_true')
    parser.add_argument('robotId', help='target robot, integer, default current robot', type=int, nargs='?')
    return parser.parse_args()


# idea: also a multi/coach mode? to control multiple robots at the same time ("van links naar rechts" dance)
# -> park, quite a narrow use case which increases complexity significantly


def set_test_worldstate(worldstate):
    # TODO: worldstate is a protobuf message, merge from json string
    test_json_string = '{"time":"2024-08-11T15:36:09.094Z","robot":{"id":2,"active":true,"position":{"x":-2,"y":-2,"rz":0},"velocity":{}},"ball":{"position":{},"velocity":{}},"teammates":[{"id":1,"active":true,"position":{"x":-4,"y":-2,"rz":0},"velocity":{}}]}'
    json_format.ParseDict(json.loads(test_json_string), worldstate)


def main(args):
    # configure logging with timestamps (milliseconds)
    loglevel = (logging.DEBUG if args.debug else logging.INFO)
    logging.basicConfig(level=loglevel, format='%(asctime)s.%(msecs)03d %(levelname)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    # check arguments
    robotId = os.getenv('TURTLE5K_ROBOTNUMBER', args.robotId)
    if not args.testmode and robotId is None:
        raise Exception('robotId not set, please provide a robotId or set TURTLE5K_ROBOTNUMBER')
    try:
        robotId = int(robotId)
    except:
        robotId = 0
    # configuration values
    joystick_config = joystick.Configuration(args.config)
    # setup joystick
    try:
        joystick_controller = joystick.JoystickController(robotId, joystick_config, args.index, args.keyboard)
        robot_interface = falcons_interface.RobotInterface(robotId)
        robot_interface.homePos[0] = joystick_config.home_x
        robot_interface.homePos[1] = joystick_config.home_y
        robot_interface.homePos[2] = joystick_config.home_rz
        if args.testmode:
            robot_interface.handle_setpoints = lambda x: logging.debug('setpoints: ' + str(json_format.MessageToJson(x, indent=None)))
            robot_interface.update_worldstate = set_test_worldstate
        joystick_controller.packet_handler = robot_interface.handle_packet
        if args.keyboard:
            logging.info('using keyboard control - press h for help, q to quit')
        joystick_controller.run()
    except Exception as e:
        raise
    finally:
        joystick_controller.cleanup()
        robot_interface.shutdown()


if __name__ == '__main__':
    main(parse_args())
