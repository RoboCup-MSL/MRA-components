#!/usr/bin/env python3

"""
For given ActionPlanning .bin files, print a summary.

Example:
    bazel run //components/falcons/action_planning/test:print /tmp/ap_A2_ACTION_CATCHBALL_FAILED_20241207T134210f066.bin

Example output:
     filename: /tmp/ap_A2_ACTION_CATCHBALL_FAILED_20241207T134210f066.bin
       action: ACTION_CATCHBALL (11)
       result: FAILED (failureReason: teammate got the ball)
     #samples: 61
        start: 2024-12-07T13:42:03.911
          end: 2024-12-07T13:42:09.930
     duration: 6.019 [s]

See also: plot.py
"""

# python modules
import sys
import argparse
import action


def parse_args(args: list) -> argparse.Namespace:
    """
    Use argparse to parse command line arguments.
    """
    descriptionTxt = __doc__
    exampleTxt = ''
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('--ticks', '-t', action='store_true', help='print tick details in table format')
    parser.add_argument('--json', '-j', action='store_true', help='full dump of data in json format')
    parser.add_argument('datafiles', nargs='+', help='data file(s) to analyze')
    return parser.parse_args(args)


def main(args: argparse.Namespace) -> None:
    # TODO: sort datafiles on timestamp?
    for f in args.datafiles:
        a = action.Action()
        a.load(f)
        a.print_info()
        if args.ticks:
            a.print_ticks()
        if args.json:
            a.print_json()
        print("")


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

