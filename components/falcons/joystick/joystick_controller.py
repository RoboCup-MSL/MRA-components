# joystick controller
# processes state of joystick_pygame (which has the main eventloop), translates to action packets

# python imports
import logging

# local imports
from joystick_core import JoystickPygame, Keyboard



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
# idea: getball in front of robot
# idea: set home location
# idea: dpad for reproducable kicker setpoint intervals
# make sure to not allow hard kick without ball
    def __init__(self, robotId, cfg, jsId=0, keyboard=False):
        self.robotId = str(robotId)
        self.cfg = cfg
        self.prev_display_string = None
        self.packet_handler = self.display_packet
        if keyboard:
            self.controller = Keyboard(cfg.frequency)
        else:
            self.controller = JoystickPygame(cfg.frequency, cfg.axis_threshold, jsId)
        self.controller.callback = self.process_state
        self.vx = 0.0
        self.vy = 0.0
        self.vrz = 0.0
        self.kicker_power = 0.0 # remember, to discharge when released
        self.prev_B_pressed = False # for toggle bh action
        self.dt = 1.0 / cfg.frequency

    def run(self):
        self.controller.run()

    def cleanup(self):
        self.controller.cleanup()

    def process_state(self, controller_state):
        logging.debug('controller_state: ' + str(controller_state))
        # analog stick input
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
        # kicker control
        kicker_power = 0.0
        if controller_state.axis_rt.x > 0:
            kicker_power = clip(controller_state.axis_rt.x * self.cfg.kicker_power_scale, self.cfg.kicker_power_max, self.cfg.kicker_power_min)
        if controller_state.axis_lt.x > 0:
            kicker_height = clip(controller_state.axis_lt.x * self.cfg.kicker_height_scale, self.cfg.kicker_height_max)
        if kicker_power > 0.0:
            self.kicker_power = max(kicker_power, self.kicker_power)
        # select action, according to this protocol:
        # * action packet is treated as a command, not as a setpoint
        #   example: getball is briefly set, then it is assumed that outside it is continuously handled
        # * action 'none' means that current action is continued and robot goes idle afterwards
        # * any state handling is done outside of the joystick controller
        # * exceptions are for analog setpoints (sticks)
        action = 'none'
        action_args = {}
        if kicker_power == 0.0 and self.kicker_power > 0.0: # first check kicker discharge
            action = 'kick'
            action_args = {'power': self.kicker_power, 'height': kicker_height}
            self.kicker_power = 0.0 # reset
        elif controller_state.buttons['A'].is_pressed:
            action = 'pass'
            if controller_state.buttons['RB'].is_pressed:
                action_args = {'target': 'home'}
            elif controller_state.buttons['LB'].is_pressed:
                action_args = {'target': 'nearestObstacle'}
            else:
                action_args = {'target': 'nearestTeammember'}
        elif controller_state.buttons['B'].is_pressed:
            if not self.prev_B_pressed:
                action = 'toggleBallhandlers' # special -- not for ActionPlanning
        elif controller_state.buttons['X'].is_pressed:
            if controller_state.buttons['LB'].is_pressed:
                action = 'getball'
                action_args = {'radius': self.cfg.getball_extended_radius}
            elif controller_state.buttons['RB'].is_pressed:
                action = 'bump'
                action_args = {'target': 'nearestTeammember'}
            else:
                action = 'getball'
                action_args = {'radius': self.cfg.getball_close_radius}
        elif controller_state.buttons['Y'].is_pressed:
            if controller_state.buttons['LB'].is_pressed:
                action = 'shoot'
                action_args = {'target': 'home'}
            elif controller_state.buttons['RB'].is_pressed:
                action = 'bump'
                action_args = {'target': 'goal'}
            else:
                action = 'shoot'
                action_args = {'target': 'goal'}
        elif controller_state.buttons['select'].is_pressed:
            action = 'keeper'
        elif controller_state.buttons['mode'].is_pressed:
            action = 'park'
            if controller_state.buttons['LB'].is_pressed:
                action = 'move'
                action_args = {'target': 'home'}
        elif self.vx != 0.0 or self.vy != 0.0 or self.vrz != 0.0:
            action = 'dash'
            action_args = {'velocity': [self.vx, self.vy, self.vrz]}
        # send/handle packet
        packet = {
            'robotId': self.robotId,
            'action': action,
            'args': action_args
        }
        self.packet_handler(packet)
        # store stuff for use in next iteration
        self.prev_B_pressed = controller_state.buttons['B'].is_pressed

    def display_packet(self, packet):
        # only on change
        display_string = (
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
