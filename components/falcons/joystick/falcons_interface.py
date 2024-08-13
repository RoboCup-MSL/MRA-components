
# use Falcons test interface to control robots

# python imports
import os
import json
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
        self.worldState = worldState.WorldState(robotId)
        self.mra_interface = mra_interface.MRAInterface(self)
        # load params from json file (TODO: our pybind interface should provide a function for this)
        action_planning_config_file = os.path.join(os.getenv('FALCONS_CODE_PATH'), 'config', 'ActionPlanningMRA.json')
        with open(action_planning_config_file, 'r') as file:
            json_data = json.load(file)
        json_format.ParseDict(json_data, self.mra_interface.params)
        # disable regular teamplay-driven execution
        self.rtdbPut('MATCH_MODE', False)

    def __del__(self):
        # restore regular teamplay-driven execution
        self.rtdbPut('MATCH_MODE', True)

    def handle_packet(self, packet : dict) -> None:
        logging.debug('controller_packet: ' + str(packet))
        # set input
        self.mra_interface.input.Clear()
        self.update_worldstate(self.mra_interface.input.worldState)
        self.mra_interface.set_input_action(packet)
        # call MRA actionplanning
        if len(packet['action']) > 0:
            r = self.mra_interface.call_action_planning()
            # dispatch setpoints
            if r == 0:
                self.handle_setpoints(self.mra_interface.output)
            # TODO: if action finished (either PASSED or FAILED), then go idle? packet producer does not know this

    def handle_setpoints(self, setpoints):
        pass

    def update_worldstate(self, world_state):
        self.worldState.update() # read from RTDB
        self.worldState.toMRA(world_state)
        logging.debug('worldstate: ' + str(json_format.MessageToJson(world_state, indent=None)))

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
        return self.rtdbStore.put(self.robotId, key, value)

    def rtdbGet(self, key, *args, **kwargs):
        return self.rtdbStore.get(self.robotId, key, *args, **kwargs)
