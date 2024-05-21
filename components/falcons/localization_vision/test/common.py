# python modules
#...

# MRA tickbin Data class
from libraries.logging.logdata import Data, FILE_ELEMENTS

# our modules
try:
    from components.falcons.localization_vision.test import pybind_ext
except ImportError:
    # maybe tick_call is not used, only Data
    pass



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

