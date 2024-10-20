#!/usr/bin/env python3

"""
Plot data from ActionPlanning to diagnose what happens at a tick.

Inputs / modes:
1. binary file a single tick: plot worldstate, target.
2. TODO action file: plot the sequence, trajectory, etc.

Example:
    bazel run //components/falcons/action_planning/test:plot /tmp/testsuite_mra_logging/tickbins/tick_FalconsActionPlanning_0.bin
"""

# python modules
import sys
import argparse
from matplotlib import pyplot as plt

# own modules
from libraries.logging.logdata import Data




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
    data = Data(args.datafile)
    action = 'unknown'
    plt.axis('off')
    plt.title(f'action={action}')
    plt.show()


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

