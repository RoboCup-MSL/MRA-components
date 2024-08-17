# joystick controller
# processes state of joystick_pygame (which has the main eventloop), translates to action packets

# python imports
import logging

# local imports
from joystick_pygame import JoystickPygame
from keyboard import KeyboardControl



def clip(v, vmax, vmin=0.0):
    return min(max(v, vmin), vmax)


class JoystickController:

    @staticmethod
    def button_text():
        return """
Controls (assuming Xbox button layout):
* movement:
    * left stick: move robot forward, backward, left, right
    * right stick: rotate robot
* ball handling:
    * B: toggle ballhandlers on/off
    * X: getball within a small action radius
    * LB+X: getball with extended action radius
    * RB+X: bump towards nearest teammate
* passing:
    * A: pass to the nearest teammate
    * LB+A: pass to the nearest obstacle
    * RB+A: pass to the home location
* shooting:
    * Y: shoot at goal
    * LB+Y: shoot at the home location
    * RB+Y: bump towards goal
* lob/kicker control:
    * LT: adjust lob height
    * RT: set shot power, release to shoot
* special actions:
    * select: activate keeper mode
    * LB+home: move the robot to configured home location
    * home: move the robot to the parking area

Notes:
* RB is right bumper, RT is right trigger, etc.
* stick movement cancels any running action
"""

    def __init__(self, robotId, cfg, jsId=0, keyboard=False):
        self.robotId = str(robotId)
        self.cfg = cfg
        self.prev_display_string = None
        self.packet_handler = self.display_packet
        self.enable_bh = False
        self.toggle_bh()
        if keyboard:
            self.controller = KeyboardControl(cfg.frequency)
        else:
            self.controller = JoystickPygame(cfg.frequency, cfg.axis_threshold, jsId)
        self.controller.buttons['B'].when_pressed = self.toggle_bh
        self.controller.callback = self.process_state
        self.vx = 0.0
        self.vy = 0.0
        self.vrz = 0.0
        self.dt = 1.0 / cfg.frequency
        self.rt = -1.0
        self.action = ''
        self.action_args = {}

    def run(self):
        self.controller.run()

    def toggle_bh(self):
        self.enable_bh = not self.enable_bh

    def process_state(self, controller_state):
        logging.debug('controller_state: ' + str(controller_state))
        def calc(current_setpoint, axis_input, speed_limit, acc_limit, deadzone):
            if abs(axis_input) < deadzone:
                return 0.0
            target = axis_input * speed_limit
            if target > current_setpoint:
                return min(current_setpoint + self.dt * acc_limit, target)
            return max(current_setpoint - self.dt * acc_limit, target)
        # TODO: clipping??
        vx = calc(self.vx, controller_state.axis_ls.x, self.cfg.motion_xy_max_speed, self.cfg.motion_xy_acceleration, self.cfg.motion_xy_deadzone)
        vy = calc(self.vy, -controller_state.axis_ls.y, self.cfg.motion_xy_max_speed, self.cfg.motion_xy_acceleration, self.cfg.motion_xy_deadzone)
        vrz = calc(self.vrz, -controller_state.axis_rs.x, self.cfg.motion_rz_max_speed, self.cfg.motion_rz_acceleration, self.cfg.motion_rz_deadzone)
        self.vx, self.vy, self.vrz = vx, vy, vrz
        self.kicker_power = 0.0
        self.kicker_height = 0.0
        if controller_state.axis_rt.x > 0:
            self.kicker_power = clip(controller_state.axis_rt.x * self.cfg.kicker_power_scale, self.cfg.kicker_power_max, self.cfg.kicker_power_min)
        if controller_state.axis_lt.x > 0:
            self.kicker_height = clip(controller_state.axis_lt.x * self.cfg.kicker_height_scale, self.cfg.kicker_height_max)
        if self.kicker_power != 0.0:
            self.action = 'kick'
            self.action_args = {'power': self.kicker_power, 'height': self.kicker_height}
        elif self.vx != 0.0 or self.vy != 0.0 or self.vrz != 0.0:
            self.action = 'move' # TODO 'dash' # local move
            self.action_args = {'velocity': [self.vx, self.vy, self.vrz]}
        elif controller_state.buttons['A'].is_pressed:
            self.action = 'pass'
            if controller_state.buttons['RB'].is_pressed:
                self.action_args = {'target': 'home'}
            elif controller_state.buttons['LB'].is_pressed:
                self.action_args = {'target': 'nearestObstacle'}
            else:
                self.action_args = {'target': 'nearestTeammember'}
        elif controller_state.buttons['X'].is_pressed:
            if controller_state.buttons['LB'].is_pressed:
                self.action = 'getball'
                self.action_args = {'radius': self.cfg.getball_extended_radius}
            elif controller_state.buttons['RB'].is_pressed:
                self.action = 'bump'
                self.action_args = {'target': 'nearestTeammember'}
            else:
                self.action = 'getball'
                self.action_args = {'radius': self.cfg.getball_close_radius}
        elif controller_state.buttons['Y'].is_pressed:
            if controller_state.buttons['LB'].is_pressed:
                self.action = 'shoot'
                self.action_args = {'target': 'home'}
            elif controller_state.buttons['RB'].is_pressed:
                self.action = 'bump'
                self.action_args = {'target': 'goal'}
            else:
                self.action = 'shoot'
                self.action_args = {'target': 'goal'}
        elif controller_state.buttons['select'].is_pressed:
            self.action = 'keeper'
        elif controller_state.buttons['mode'].is_pressed:
            self.action = 'park'
            if controller_state.buttons['LB'].is_pressed:
                self.action = 'move'
                self.action_args = {'target': 'home'}
        elif self.vx == 0.0 and self.vy == 0.0 and self.vrz == 0.0 and self.action == 'move':
            self.action_args = {}
            self.action = 'stop'
        # send/handle packet
        packet = {
            'robotId': self.robotId,
            'action': self.action,
            'enableBallHandlers': self.enable_bh,
            'args': self.action_args
        }
        self.packet_handler(packet)

    def display_packet(self, packet):
        # only on change
        display_string = (
            f"bh={'on' if packet['enableBallHandlers'] else 'off':3} "
            f"action={packet['action']:10s}"
        )
        def float_formatter(v):
            if isinstance(v, str):
                return v
            elif isinstance(v, list):
                return ['{:6.2f}'.format(x) for x in v]
            return '{:6.2f}'.format(v)
        if len(packet['args']):
            args_dict_formatted = {k: float_formatter(v) for k, v in packet['args'].items()}
            display_string += f'  args={args_dict_formatted}'
        if display_string != self.prev_display_string:
            logging.info(display_string)
            self.prev_display_string = display_string
