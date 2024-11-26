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
import struct
from datetime import datetime
from matplotlib import pyplot as plt

# MRA modules
from libraries.logging.logdata import Data
from libraries.plotting import plot_worldstate, plot_field, plot_target
import components.falcons.action_planning.interface.State_pb2 as State_pb2



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
        main_tickbin(args)
    else:
        main_actionfile(args)


def protobuf_enum2str(pb_object, field_name) -> str:
    """
    Get the enum string value of a enum in a protobuf object.
    """
    return pb_object.DESCRIPTOR.fields_by_name[field_name].enum_type.values_by_number[getattr(pb_object, field_name)].name


def check_clean_execution(history: State_pb2.ActionHistory) -> str:
    """
    Check if action execution was clean.

    A clean action has only the final tick result as PASSED or FAILED, all other ticks RUNNING.

    A dirty action can be
     * early PASSED/FAILED, continue to run anyway (teamplay protocol issue?)
     * somewhere an INVALID tick occurred
     * all ticks RUNNING, teamplay decided to switch to another action

    Return a string to describe the action cleanliness, for example
    * 'clean'
    * 'dirty: tick 3 already returned PASSED'
    """
    RUNNING = 2
    # store the result of each tick as single character
    details = ''
    for sample in history.samples:
        details += protobuf_enum2str(sample.output, 'actionresult')[0]
    print(details)
    # check if any tick was INVALID
    idx = details.find('I')
    if idx != -1:
        return f'dirty: tick {idx} returned INVALID'
    # check if action actually finished with PASSED or FAILED
    if history.samples[-1] == RUNNING:
        return 'incomplete: last tick returned RUNNING'
    # check for early result
    for (it, c) in enumerate(details[:-1]):
        if c != 'R':
            early_result = protobuf_enum2str(history.samples[it].output, 'actionresult')
            return f'dirty: tick {it} already returned {early_result}'
    return 'clean'


def main_actionfile(args: argparse.Namespace) -> None:
    """
    Plot an action file.
    """
    # load
    raw_data = None
    with open(args.datafile, 'rb') as f:
        n = struct.unpack('i', f.read(4))[0]
        raw_data = f.read(n)
    # create history object
    history = State_pb2.ActionHistory()
    history.MergeFromString(raw_data)
    # print some info
    print_actionfile_info(history)
    # setup the plotter
    # TODO plot_actionfile(history)


def print_actionfile_info(history: State_pb2.ActionHistory) -> None:
    """
    Print some info about the action.
    """
    action_type = history.type
    action_type_str = protobuf_enum2str(history, 'type')
    print(f'     action: {action_type_str} ({action_type})')
    action_result = history.samples[-1].output.actionresult
    action_result_str = protobuf_enum2str(history.samples[-1].output, 'actionresult')
    print(f'     result: {action_result_str} ({action_result})')
    print(f'   #samples: {len(history.samples)}')
    # check if action execution was clean (some actions are not interesting to check)
    if action_type_str not in ['ACTION_STOP', 'ACTION_MOVE', 'ACTION_KEEPER']:
        print(f'     ?clean: {check_clean_execution(history)}')
    # timestamps, duration
    start_timestamp = datetime.fromtimestamp(history.samples[0].timestamp.seconds + history.samples[0].timestamp.nanos / 1e9)
    end_timestamp = datetime.fromtimestamp(history.samples[-1].timestamp.seconds + history.samples[-1].timestamp.nanos / 1e9)
    # get duration as seconds
    duration = (end_timestamp - start_timestamp).total_seconds()
    start_timestamp_str = start_timestamp.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
    end_timestamp_str = end_timestamp.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
    print(f'      start: {start_timestamp_str}')
    print(f'        end: {end_timestamp_str}')
    print(f'   duration: {duration:.3f} [s]')


def main_tickbin(args: argparse.Namespace) -> None:
    """
    Plot a single tick .bin file.
    """
    data = Data(args.datafile)
    action = data.input.action
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plot_field(ax)
    plot_worldstate(ax, data.input.worldState)
    targetpos = data.output.setpoints.move.target
    plot_target(ax, targetpos, data.input.worldState.robot)
    plt.title(f'action={action}')
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    plt.show()


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

