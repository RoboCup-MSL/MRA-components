#!/usr/bin/env python3

"""
Given actionplanning files (.bin) in a folder, combine them for plotting in Perfetto/Catapult.
"""

# python modules
import sys
import argparse
import glob
import json

# our test modules
import components.falcons.action_planning.test.action as action

# constants
DEFAULT_OUTPUT = '/tmp/actions.json'


def parse_args(args: list) -> argparse.Namespace:
    """
    Use argparse to parse command line arguments.
    """
    descriptionTxt = __doc__
    exampleTxt = ''
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('-o', '--output', type=str, default=DEFAULT_OUTPUT, help='output json file name')
    parser.add_argument('folder', help='data folder to load')
    return parser.parse_args(args)


def load(folder: str) -> None:
    """
    Load the action planning files from the folder.
    """
    files = glob.glob(folder + '/*.bin')
    result_data = []
    for f in files:
        robot = 'robot ' + f.split('_A')[1].split('_')[0] # guess robot id from file name
        a = action.Action()
        a.load(f)
        result_data += a.timeline_entries(robot)
    return result_data


def write_json_file(data: list, output: str) -> None:
    """
    Write json file, one element per row, such that editors will not choke on the row size.
    """
    with open(output, 'w') as f:
        f.write('[\n')
        rows = [json.dumps(elem) for elem in data]
        f.write(',\n'.join(rows))
        f.write('\n]\n')


def main(args: argparse.Namespace) -> None:
    """
    Make the plot(s).
    """
    data = load(args.folder)
    write_json_file(data, args.output)


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

