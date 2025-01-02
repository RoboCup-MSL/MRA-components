#!/usr/bin/env python3

"""
Plot data from ActionPlanning to diagnose what happens at a tick or during the tick sequence.

Inputs / modes:
1. binary file a single tick: plot worldstate, target.
2. action file: plot the sequence, trajectory, etc.

Examples:
    bazel run //components/falcons/action_planning/test:plot /tmp/testsuite_mra_logging/tickbins/tick_FalconsActionPlanning_0.bin
    bazel run //components/falcons/action_planning/test:plot /tmp/ap_ACTION_GETBALL_PASSED_20241121T193131f746.bin

For the interactive plot, use the left/right arrow keys to navigate through the samples, and press q to quit.

See also: print.py.
"""

# python modules
import sys
import argparse

# our test modules
import components.falcons.action_planning.test.action as action


def parse_args(args: list) -> argparse.Namespace:
    """
    Use argparse to parse command line arguments.
    """
    descriptionTxt = __doc__
    exampleTxt = ''
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('datafile', help='data file to load')
    return parser.parse_args(args)


def main(args: argparse.Namespace) -> None:
    """
    Make the plot.
    """
    if args.datafile.endswith('.bin') and 'tick' in args.datafile:
        a = action.ActionTick()
        a.load(args.datafile)
        a.plot()
    else:
        a = action.Action()
        a.load(args.datafile)
        a.print_info()
        a.plot()


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

