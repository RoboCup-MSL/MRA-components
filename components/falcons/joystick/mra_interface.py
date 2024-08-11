
# python imports
# ...

# local imports
from components.falcons.joystick.pybind import action_planning_pybind

# other MRA imports
from components.falcons.action_planning.interface import Input_pb2, Params_pb2, State_pb2
from datatypes import ActionType_pb2



class MRAInterface():
    def __init__(self):
        self.input = Input_pb2.Input()
        self.params = Params_pb2.Params() # client should overrule from config json file, or get error code 1 at first tick
        self.state = State_pb2.State()

    def set_input_worldstate(self, worldstate):
        self.input.worldstate = worldstate

    def set_input_action(self, packet : dict):
        # return if no action set
        if packet['action'] == '':
            return
        # convert to enum action value
        action_str = 'ACTION_' + packet['action'].upper()
        self.input.action.type = getattr(ActionType_pb2, action_str)
        # TODO: resolve strings to coordinates using mra_input.worldstate
        # TODO: switch on packet['action'] and assign subfield accordingly

    def call_action_planning(self, check : bool = True) -> int:
        """Call the action_planning tick."""
        # call tick
        return_tuple = action_planning_pybind.tick(
            self.input,
            self.params,
            self.state)
        # check return code
        return_code = return_tuple[0]
        if check:
            if return_code != 0:
                raise RuntimeError('tick call failed with return code {:d}'.format(return_code))
        # store results
        self.state = return_tuple[1]
        self.output = return_tuple[2]
        self.local = return_tuple[3]
        return return_code
