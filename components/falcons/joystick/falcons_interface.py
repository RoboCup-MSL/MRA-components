
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
        self.pp_pid = subprocess.Popen(['processStart', 'A'+rs, 'pp']).pid
        logging.info('started pathPlanning node')

    def processhack_end(self):
        os.kill(self.pp_pid, signal.SIGTERM)
        logging.info('killed pathPlanning node') # TODO: make this more robust, restartwrapper and such are in the way

    def handle_packet(self, packet : dict) -> None:
        logging.debug('controller_packet: ' + str(packet))
        # set input
        self.mra_interface.input.Clear()
        ws_ok = self.update_worldstate(self.mra_interface.input.worldState)
        if not ws_ok:
            # warn only once per second
            if not hasattr(self, 'last_ws_warning') or (datetime.datetime.now() - self.last_ws_warning).total_seconds() > 1:
                self.last_ws_warning = datetime.datetime.now()
                logging.warning('failed to update worldstate, is robot SW activated?')
            return
        self.mra_interface.set_input_action(packet)
        # prep setpoints
        setpoints = None
        # call MRA actionplanning
        mra_ok = 0
        if len(packet['action']) > 0:
            # special case: robot velocity setpoint (sticks) arrives via action 'move' with arguments 'velocity'
            # TODO: consider if ActionPlanning should also cover it, using action DASH?
            if packet['action'] == 'move' and 'velocity' in packet['args']:
                setpoints = self.mra_interface.output.setpoints # get the correct protobuf type
                setpoints.Clear()
                setpoints.velocity.x = packet['args']['velocity'][0]
                setpoints.velocity.y = packet['args']['velocity'][1]
                setpoints.velocity.rz = packet['args']['velocity'][2]
            else:
                # regular handling: call action planning
                if 0 == self.mra_interface.call_action_planning():
                    setpoints = self.mra_interface.output.setpoints
        # dispatch setpoints
        if setpoints:
            self.handle_setpoints(setpoints)
        # TODO: if action finished (either PASSED or FAILED), then go idle? packet producer does not know this

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
        except:
            return False
        logging.debug('worldstate: ' + str(json_format.MessageToJson(world_state, indent=None)))
        return True

    def resolveTarget(self, target):
        if target == 'home':
            return self.homePos
        elif target == 'goal':
            return self.goalPos
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
