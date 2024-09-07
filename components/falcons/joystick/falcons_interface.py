
# use Falcons test interface to control robots

# python imports
import os
import subprocess
import signal
import json
import datetime
import logging
from google.protobuf import json_format

# local imports
import mra_interface

# falcons imports
import falconspy # modifies sys.path to enable next imports
import falconsrtdb
import worldState # from Falcons WorldModel
import EnvironmentField
# import falconsrtdb
# from sharedTypes import sharedTypes
# write RTDB setpoints, etc.



def extra_tracing():
    # slap some tracing decorators onto our code, automagically!
    # (based on autologging extensions in my repo https://github.com/janfeitsma/extendedlogging)
    import tracing


class RobotInterface():
    def __init__(self, robotId):
        self.robotId = robotId
        self.rtdbStore = falconsrtdb.FalconsRtDBStore(readonly=False)
        self.rtdbStore.refresh_rtdb_instances()
        self.homePos = [0, 0, 0]
        self.goalPos = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI(EnvironmentField.P_OPP_GOALLINE_CENTER)
        # setup WorldState = Falcons WorldModel listener
        self.worldState = worldState.WorldState(robotId)
        # setup MRA interface
        self.mra_interface = mra_interface.MRAInterface(self)
        # load params from json file (TODO: our pybind interface should provide a function for this)
        action_planning_config_file = os.path.join(os.getenv('FALCONS_CODE_PATH'), 'config', 'ActionPlanningMRA.json')
        with open(action_planning_config_file, 'r') as file:
            json_data = json.load(file)
        json_format.ParseDict(json_data, self.mra_interface.params)
        # disable regular teamplay-driven execution of Falcons software
        self.rtdbPut('MATCH_MODE', False)
        # process hack: pathPlanning is not started by default
        self.processhack_start()
        # running action
        self.current_action = None
        self.current_action_args = None

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        # TODO: make sure this get called at process exit, not only regular shutdown, but also when the process dies
        # restore regular teamplay-driven execution
        self.rtdbPut('MATCH_MODE', True)
        self.processhack_end()

    def processhack_start(self):
        rs = str(self.robotId)
        os.environ['TURTLE5K_ROBOTNUMBER'] = rs
        pp_cmd = subprocess.check_output(['processCmd', 'pp']).strip().decode('utf-8') # something like : '/home/jan/falcons/code/bazel-bin/packages/pathPlanning/pathplanningNode'
        logfile_base = '/tmp/joystick_falcons'
        self.pp_proc = subprocess.Popen([pp_cmd], stdout=open(logfile_base + '_stdout_' + rs + '.txt', 'w'), stderr=open(logfile_base + '_stderr_' + rs + '.txt', 'w'))
        logging.info('started pathPlanning node')

    def processhack_end(self):
        self.pp_proc.kill()
        logging.info('killed pathPlanning node')

    def poke_action(self, action, action_args):
        # check for new action; action 'none' signals to continue any current unfinished action
        if action != 'none':
            if action != self.current_action:
                self.on_action_start(action, action_args)
        # call MRA action planning, use current action memory
        setpoints, actionresult = None, None
        if self.current_action:
            self.mra_interface.set_input_action(self.current_action, self.current_action_args)
            if 0 == self.mra_interface.call_action_planning(): # 0 is OK
                setpoints = self.mra_interface.output.setpoints
                # convert int actionresult to protobuf enum string
                # TODO: this is not ideal, sensitive to enum change ... better to use the enum value directly
                actionresult = ['INVALID', 'FAILED', 'RUNNING', 'PASSED'][self.mra_interface.output.actionresult]
        if actionresult in ['PASSED', 'FAILED'] and self.current_action:
            self.on_action_end(actionresult)
            self.stop_moving(setpoints)
        return setpoints, actionresult

    def on_action_start(self, action, action_args):
        if action not in ['stop']:
            logging.info('action {} starting (args {})'.format(action, str(action_args)))
        self.current_action = action
        self.current_action_args = action_args

    def on_action_end(self, actionresult):
        if self.current_action not in ['stop']:
            logging.info('action {} finished: {}'.format(self.current_action, actionresult))
        self.current_action = None
        self.current_action_args = None

    def stop_moving(self, setpoints):
        setpoints.velocity.x = 0
        setpoints.velocity.y = 0
        setpoints.velocity.rz = 0
        self.current_action == None

    def handle_packet(self, packet : dict) -> None:
        logging.debug('controller_packet: ' + str(packet))
        # prepare
        self.mra_interface.input.Clear()
        self.update_worldstate(self.mra_interface.input.worldState)
        setpoints = self.mra_interface.output.setpoints # get the correct protobuf type
        setpoints.Clear()
        # handle packets which should not be handled by ActionPlanning (yet)
        if packet['action'] == 'dash' and 'velocity' in packet['args']:
            setpoints.velocity.x = packet['args']['velocity'][0]
            setpoints.velocity.y = packet['args']['velocity'][1]
            setpoints.velocity.rz = packet['args']['velocity'][2]
            self.current_action = 'dash'
        elif packet['action'] == 'none' and self.current_action == 'dash':
            self.stop_moving(setpoints)
        elif packet['action'] == 'toggleBallHandlers':
            setpoints.bh.enabled = not self.getBallHandlersEnabled()
        # handle regular actions
        elif len(packet['action']) > 0:
            setpoints, actionresult = self.poke_action(packet['action'], packet['args'])
        # dispatch setpoints
        if setpoints:
            self.handle_setpoints(setpoints)

    def handle_setpoints(self, setpoints):
        logging.debug('handle_setpoints: ' + str(setpoints))
        # there is a race condition at (sim) init time with execution setting MATCH_MODE ... lets check always
        if True == self.rtdbGet('MATCH_MODE', timeout=None).value:
            logging.warning('MATCH_MODE is enabled, cannot overrule setpoints')
            return
        if setpoints.HasField('bh'):
            self.rtdbPut('BALLHANDLERS_SETPOINT', setpoints.bh.enabled)
        if setpoints.HasField('move'):
            move = setpoints.move
            motionSetpoint = {
                'action': 5 if move.stop else 1, # TODO: not good to hardcode these enum values
                'position': [
                    move.target.position.x,
                    move.target.position.y,
                    move.target.position.rz
                ],
                'motionType': 1 # 1 is normal, 2 is with ball
            }
            self.rtdbPut('MOTION_SETPOINT', motionSetpoint)
            # it is assumed that pathPlanning is actively iterating (via waitForPut), so we do not need to call it here
            # default Falcons SW deployment gives control to motionPlanning, which in turn calls pathPlanning iterate
        if setpoints.HasField('velocity'):
            velocity = setpoints.velocity
            self.rtdbPut('ROBOT_VELOCITY_SETPOINT', [velocity.x, velocity.y, velocity.rz])
        if setpoints.HasField('shoot'):
            shoot = setpoints.shoot
            shootSetpoint = {
                'position': {
                    'x': shoot.pos_x,
                    'y': shoot.pos_y,
                    'z': shoot.pos_z
                },
                'shootPhase': shoot.phase,
                'shootType': shoot.type
            }
            self.rtdbPut('SHOOT_SETPOINT', shootSetpoint)

    def update_worldstate(self, world_state):
        try:
            self.worldState.update() # read from RTDB
            self.worldState.toMRA(world_state)
            success = True
        except:
            success = False
        if success:
            logging.debug('worldstate: ' + str(json_format.MessageToJson(world_state, indent=None)))
        else:
            # warn only once every few seconds
            num_seconds = 3
            if not hasattr(self, 'last_ws_warning') or (datetime.datetime.now() - self.last_ws_warning).total_seconds() > num_seconds:
                self.last_ws_warning = datetime.datetime.now()
                logging.warning('failed to update worldstate, is robot SW activated?')
        return success

    def resolveTarget(self, target):
        if target == 'home':
            return self.homePos
        elif target == 'goal':
            return (self.goalPos.x, self.goalPos.y, 0)
        elif target == 'nearestObstacle':
            xy = self.worldState.nearestObstacle()
            return (xy.x, xy.y, 0)
        elif target == 'nearestTeammember':
            xy = self.worldState.nearestTeammember()
            return (xy.x, xy.y, 0)
        else:
            return target

    def getBallHandlersEnabled(self) -> bool:
        r = self.rtdbGet('BALLHANDLERS_SETPOINT', timeout=None)
        if r != None:
            return r.value
        return False

    def rtdbPut(self, key, value):
        logging.debug('rtdbPut: ' + key + '=' + str(value))
        return self.rtdbStore.put(self.robotId, key, value)

    def rtdbGet(self, key, *args, **kwargs):
        return self.rtdbStore.get(self.robotId, key, *args, **kwargs)
