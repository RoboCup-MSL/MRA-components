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
import logging

# local imports
from components.falcons.joystick import joystick
import falcons_interface


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
    joystick_controller = joystick.JoystickController(robotId, joystick_config, args.index)
    robot_interface = falcons_interface.RobotInterface(robotId)
    if args.testmode:
        robot_interface.handle_setpoints = print
    joystick_controller.packet_handler = robot_interface.handle_packet
    joystick_controller.run()


if __name__ == '__main__':
    main(parse_args())
