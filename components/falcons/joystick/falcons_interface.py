
# use Falcons test interface to control robots

# python imports
# ...

# local imports
import mra_interface

# falcons imports
# TODO
# import falconspy
# import falconsrtdb
# from sharedTypes import sharedTypes
# write RTDB setpoints, etc.




class RobotInterface():
    def __init__(self, robotId):
        self.robotId = robotId
        self.mra_interface = mra_interface.MRAInterface()
        #self.mra_interface.params = # TODO load from falcons config json file as protobuf message

    def handle_packet(self, packet):
        print(packet)
        # set input
        self.update_worldstate(self.mra_interface.input.worldState)
        self.mra_interface.set_input_action(packet)
        # call MRA actionplanning
        r = self.mra_interface.call_action_planning()
        # dispatch setpoints
        if r == 0:
            self.handle_setpoints(self.mra_interface.data.output)
        # TODO: if action finished (either PASSED or FAILED), then go idle? packet producer does not know this

    def handle_setpoints(self, setpoints):
        print(setpoints)

    def update_worldstate(self, world_state):
        # TODO: get worldstate from RTDB as MRA WorldState object
        world_state.robot.position.x = 3.0
