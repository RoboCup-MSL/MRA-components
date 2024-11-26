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


class Action:
    """
    Class to handle an action file, covering a series of samples, typically leading to a result PASSED/FAILED.
    """
    def __init__(self):
        self.data = None

    def load(self, datafile: str):
        """
        Load the data from a file.
        """
        with open(datafile, 'rb') as f:
            n = struct.unpack('i', f.read(4))[0]
            raw_data = f.read(n)
        self.data = State_pb2.ActionHistory()
        self.data.MergeFromString(raw_data)

    def plot(self):
        self.setup_plotter()

    def print_info(self):
        """
        Print some info to stdout about the action.
        """
        action_type = self.data.type
        action_type_str = protobuf_enum2str(self.data, 'type')
        print(f'     action: {action_type_str} ({action_type})')
        action_result = self.data.samples[-1].output.actionresult
        action_result_str = protobuf_enum2str(self.data.samples[-1].output, 'actionresult')
        print(f'     result: {action_result_str} ({action_result})')
        print(f'   #samples: {len(self.data.samples)}')
        #if action_type_str not in ['ACTION_STOP', 'ACTION_MOVE', 'ACTION_KEEPER']:
        #    print(f'     ?clean: {check_clean_execution(self.history)}')
        start_timestamp = datetime.fromtimestamp(self.data.samples[0].timestamp.seconds + self.data.samples[0].timestamp.nanos / 1e9)
        end_timestamp = datetime.fromtimestamp(self.data.samples[-1].timestamp.seconds + self.data.samples[-1].timestamp.nanos / 1e9)
        duration = (end_timestamp - start_timestamp).total_seconds()
        start_timestamp_str = start_timestamp.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        end_timestamp_str = end_timestamp.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        print(f'      start: {start_timestamp_str}')
        print(f'        end: {end_timestamp_str}')
        print(f'   duration: {duration:.3f} [s]')

    def setup_plotter(self):
        def on_key_press(event):
            if event.key == 'right':
                on_key_press.idx = (on_key_press.idx + 1) % len(self.data.samples)
            elif event.key == 'left':
                on_key_press.idx = (on_key_press.idx - 1) % len(self.data.samples)
            ax.clear()
            plot_field(ax)
            self.plot_tick_io(self.data.samples[on_key_press.idx].input, self.data.samples[on_key_press.idx].output, ax)
            fig.canvas.draw()

        on_key_press.idx = 0
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_field(ax)
        fig.canvas.mpl_connect('key_press_event', on_key_press)
        self.plot_tick_io(self.data.samples[0].input, self.data.samples[0].output, ax)
        plt.title(f'action=TODO')
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        plt.show()

    def plot_tick_io(self, input, output, ax: plt.Axes):
        plot_worldstate(ax, input.worldState)
        targetpos = output.setpoints.move.target
        plot_target(ax, targetpos, input.worldState.robot)


class ActionTick:
    """
    Class to handle a single tick of an action.
    """
    def __init__(self):
        self.data = None

    def load(self, datafile: str):
        """
        Load the data from a file.
        """
        self.data = Data(datafile)

    def plot(self, ax: plt.Axes = None):
        """
        Plot the situation.

        If ax is None, create a new figure.
        """
        if self.data is None:
            raise Exception('no data loaded')
        # if ax is given, then only plot the input/output and assume the rest of the figure is already taken care of
        if not ax is None:
            self.plot_io(self.data.input, self.data.output, ax)
            return
        # no ax is given, so create new figure
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_field(ax)
        self.plot_io(self.data.input, self.data.output, ax)
        action = self.data.input.action
        plt.title(f'action={action}')
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        plt.show()

    def plot_io(self, input, output, ax: plt.Axes):
        """
        Only plot input and output.
        """
        plot_worldstate(ax, input.worldState)
        targetpos = output.setpoints.move.target
        plot_target(ax, targetpos, input.worldState.robot)


def main(args: argparse.Namespace) -> None:
    """
    Make the plot.
    """
    if args.datafile.endswith('.bin') and 'tick' in args.datafile:
        a = ActionTick()
        a.load(args.datafile)
        a.plot()
    else:
        a = Action()
        a.load(args.datafile)
        a.print_info()
        a.plot()


def protobuf_enum2str(pb_object, field_name) -> str:
    """
    Get the enum string value of a enum in a protobuf object.
    """
    return pb_object.DESCRIPTOR.fields_by_name[field_name].enum_type.values_by_number[getattr(pb_object, field_name)].name
    # TODO: this utility function should perhaps be shared at a common level


# TODO: this function to determine action 'verdict' string should perhaps be moved into C++ code, at action end evaluation
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


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

