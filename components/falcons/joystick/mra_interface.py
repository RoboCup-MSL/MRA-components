
# python imports
# ...

# local imports
from components.falcons.joystick.pybind import action_planning_pybind

# other MRA imports
from components.falcons.action_planning.interface import Input_pb2, Params_pb2, State_pb2, Output_pb2
from datatypes import ActionType_pb2



class MRAInterface():
    def __init__(self, robot_interface):
        self.robot_interface = robot_interface
        self.input = Input_pb2.Input()
        self.params = Params_pb2.Params() # client should overrule from config json file, or get error code 1 at first tick
        self.state = State_pb2.State()
        self.output = Output_pb2.Output()

    def set_input_worldstate(self, worldstate):
        self.input.worldstate = worldstate

    def set_input_action(self, action : str, action_args : dict):
        # return if no action set
        if action == '':
            return
        # convert to enum action value
        # TODO: implement all actions after interface is agreed upon
        action_str = 'ACTION_' + action.upper()
        self.input.action.type = getattr(ActionType_pb2, action_str)
        # switch on action and assign subfield accordingly
        if action == 'stop':
            self.input.action.stop.ballHandlersEnabled = self.robot_interface.getBallHandlersEnabled()
        elif action == 'move':
            # TODO self.input.action.move.motionTarget.position = self.robot_interface.resolveTarget(action_args['target'])
            self.input.action.move.motionType = 0 # TODO dribble?
            self.input.action.move.ballHandlersEnabled = self.robot_interface.getBallHandlersEnabled()
        elif action == 'kick':
            pass
        elif action == 'pass':
            pos = getattr(self.input.action, 'pass').target.position
            pos.x, pos.y, pos.rz = self.robot_interface.resolveTarget(action_args['target'])
        elif action == 'shoot':
            pos = self.input.action.shoot.target.position
            pos.x, pos.y, pos.rz = self.robot_interface.resolveTarget(action_args['target'])
        elif action == 'lob':
            pos = self.input.action.lob.target.position
            pos.x, pos.y, pos.rz = self.robot_interface.resolveTarget(action_args['target'])
        elif action == 'getball':
            self.input.action.getball.motionType = 0
            self.input.action.getball.radius = action_args['radius']
        elif action == 'shield':
            self.input.action.shield.CopyFrom(ActionShieldInputs())
        elif action == 'keeper':
            self.input.action.keeper.CopyFrom(ActionKeeperInputs())
        elif action == 'intercept':
            self.input.action.intercept.CopyFrom(ActionInterceptInputs())
        elif action == 'park':
            self.input.action.park.CopyFrom(MRA.FalconsActionPark.Input())

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
        self.diagnostics = return_tuple[3]
        return return_code
