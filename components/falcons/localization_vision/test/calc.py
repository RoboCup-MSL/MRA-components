#!/usr/bin/env python3

"""
Single calculation on data from .bin file.

Modes:
* calc (default): a single calc function call, via manual mode.
* converge: the simplex algorithm (single-tracker) is run to converge on a local optimum.
* tick: a full tick is done, which typically also includes running multiple trackers multi-threaded, some randomized.

Actually, in all 3 modes, the same tick function is called over the python bridge. This script just takes care of manipulating the inputs/params.
Params apply to some extent to all 3 modes.
State is only relevant for tick mode.
"""


# python modules
import sys
import argparse

# own modules
import common
from components.falcons.localization_vision.test import pybind_ext

# constants
MODES = ['calc', 'converge', 'tick']



def parse_args(args: list) -> argparse.Namespace:
    """
    Use argparse to parse command line arguments.
    """
    descriptionTxt = __doc__
    exampleTxt = ''
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('-g', '--guess', help='initial guess, comma-separated string x,y,rz')
    parser.add_argument('-m', '--mode', help='which calculation mode to use', default=MODES[0], choices=MODES)
    parser.add_argument('datafile', help='data file (.bin) to load')
    # TODO: parameter overrule(s)? latest design params, or from some yaml file, or ...
    # TODO: option to wipe state
    return parser.parse_args(args)


def parse_guess(guess_arg, protobuf_type):
    if guess_arg is None:
        return
    result = protobuf_type()
    try:
        guess_parts = guess_arg.split(',')
        result.x = float(guess_parts[0])
        result.y = float(guess_parts[1])
        result.rz = float(guess_parts[2])
    except:
        raise RuntimeError('something went wrong while parsing "guess" argument')
    return result


def main(args: argparse.Namespace) -> None:
    """
    Do the calculation.
    """
    # load the data
    data = common.Data(args.datafile)
    # options / tweaks

    # overrule initial guess
    guess_pose = parse_guess(args.guess, data.input.guess.__class__)
    if args.guess:
        data.input.guess.x = guess_pose.x
        data.input.guess.y = guess_pose.y
        data.input.guess.rz = guess_pose.rz

    # set manual mode, to disable the random/threading stuff
    if args.mode != 'tick':
        data.params.solver.manual.enabled = True
        data.params.solver.manual.converge = (args.mode == 'converge')
        if args.guess:
            data.params.solver.manual.pose.x = guess_pose.x
            data.params.solver.manual.pose.y = guess_pose.y
            data.params.solver.manual.pose.rz = guess_pose.rz

    # call tick
    common.tick_call(data, check=True)

    # print result
    print('tick result: ' + common.tick_info(data))


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

