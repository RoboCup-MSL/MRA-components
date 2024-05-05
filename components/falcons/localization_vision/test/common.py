# python modules
import struct
import json
import google.protobuf.timestamp_pb2
from google.protobuf import json_format

# our modules
from components.falcons.localization_vision.interface import Input_pb2, Params_pb2, State_pb2, Output_pb2, Local_pb2
try:
    from components.falcons.localization_vision.test import pybind_ext
except ImportError:
    # maybe tick_call is not used, only Data
    pass

# data elements
# mutable elements such as state get also a _before and _after variant, as stored in tick file
DATA_ELEMENTS = (
    ('input', False),
    ('params', False),
    ('state', True),
    ('output', False),
    ('local', False))
FILE_ELEMENTS = (
    'input',
    'params',
    'state_before',
    'output',
    'local',
    'state_after')
# these are all protobuf objects


class Data():

    def __init__(self, filename = None):
        self.reset()
        if filename:
            self.loadTickFile(filename)

    def reset(self):
        self.t = google.protobuf.timestamp_pb2.Timestamp()
        for (key, mutable) in DATA_ELEMENTS:
            msg = eval('{0}_pb2.{0}()'.format(key.capitalize())) # example Input_pb2.Input()
            setattr(self, key, msg)
            if mutable:
                setattr(self, key+'_before', msg)
                setattr(self, key+'_after', msg)

    def setDefaultParams(self):
        self.readJsonFileIntoMessage('components/falcons/localization_vision/interface/DefaultParams.json', self.params)

    def readJsonFileIntoMessage(self, filename, msg):
        with open(filename, 'r') as file:
            json_data = json.load(file)
        json_format.ParseDict(json_data, msg)

    def loadTickFile(self, filename):
        # setup parameters as follows:
        # 1. use defaultParams, because those are consistent with current code
        # 2. overrule with values from tick bin, which may be produced using older code
        self.setDefaultParams()
        # data is serialized, see MRA libraries/logging/logging.hpp dumpToFile
        # each protobuf object is an int (#bytes) followed by serialized protobuf bytes
        bytedata = None
        with open(filename, 'rb') as f:
            for key in FILE_ELEMENTS:
                # read next
                n = struct.unpack('i', f.read(4))[0]
                data = f.read(n)
                getattr(self, key).MergeFromString(data)



def tick_call(data : Data, check : bool = True) -> int:
    """Call the localization_vision tick."""
    # call tick
    return_tuple = pybind_ext.tick(
        data.input,
        data.params,
        data.state_before)
    # check return code
    return_code = return_tuple[0]
    if check:
        if return_code != 0:
            raise RuntimeError('tick call failed with return code {:d}'.format(return_code))
    # store results
    data.state_after = return_tuple[1]
    data.output = return_tuple[2]
    data.local = return_tuple[3]
    return return_code


def tick_info(data : Data) -> str:
    """Create an informative string (json) of what happened in a localization_vision tick."""
    result = 'N/A'
    if len(data.output.candidates):
        c = data.output.candidates[0]
        numberOfTries = data.local.numberOfTries[0]
        result = '{"pose":{' \
            + '"x":{:.6f},"y":{:.6f},"rz":{:.6f}'.format(c.pose.x, c.pose.y, c.pose.rz) \
            + '},"confidence":' + '{:.6f}'.format(c.confidence) \
            + ',"numberOfTries":' + '{:d}'.format(numberOfTries) \
            + '}'
    return result

