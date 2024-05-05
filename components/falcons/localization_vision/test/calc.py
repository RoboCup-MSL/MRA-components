#!/usr/bin/env python3

"""
Single fit&confidence calculation on data from .bin file.
"""


# python modules
import sys
import argparse

# own modules
import common
from components.falcons.localization_vision.test import pybind_ext



def parse_args(args: list) -> argparse.Namespace:
    """
    Use argparse to parse command line arguments.
    """
    descriptionTxt = __doc__
    exampleTxt = ''
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('datafile', help='data file (.bin) to load')
    # TODO: parameter overrule(s)? latest design params, or from some yaml file, or ...
    # TODO: option to wipe state
    # TODO: option to set initial guess
    return parser.parse_args(args)


def main(args: argparse.Namespace) -> None:
    """
    Do the calculation.
    """
    # load the data
    data = common.Data(args.datafile)
    # options / tweaks
    
    # TODO: data.input.guess

    # TODO: set manual mode, to disable the guessing/threading stuff
    
    # call tick
    common.tick_call(data, check=True)

    # print result
    print('tick result: ' + common.tick_info(data))


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

