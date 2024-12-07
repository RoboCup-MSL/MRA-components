# python modules
import struct
from datetime import datetime
from matplotlib import pyplot as plt

# MRA modules
from libraries.logging.logdata import Data
from libraries.plotting import plot_worldstate, plot_field, plot_target
import components.falcons.action_planning.interface.State_pb2 as State_pb2



def protobuf_enum2str(pb_object, field_name) -> str:
    """
    Get the enum string value of a enum in a protobuf object.
    """
    return pb_object.DESCRIPTOR.fields_by_name[field_name].enum_type.values_by_number[getattr(pb_object, field_name)].name
    # TODO: this utility function should perhaps be shared at a common level


class Action:
    """
    Class to handle an action file, covering a series of samples, typically leading to a result PASSED/FAILED.
    """
    def __init__(self):
        self.filename = None
        self.data = None
        self.tickplotter = ActionTick()
        self.sample_idx = 0
        self.fig = None
        self.ax = None

    def load(self, datafile: str):
        """
        Load the data from a file.
        """
        self.filename = datafile
        with open(datafile, 'rb') as f:
            n = struct.unpack('i', f.read(4))[0]
            raw_data = f.read(n)
        self.data = State_pb2.ActionHistory()
        self.data.MergeFromString(raw_data)
        self.action_type_str = protobuf_enum2str(self.data, 'type')

    def on_key_press(self, event):
        """
        Keypress event handler.
        """
        if event.key == 'right':
            self.sample_idx = (self.sample_idx + 1) % len(self.data.samples)
        elif event.key == 'left':
            self.sample_idx = (self.sample_idx - 1) % len(self.data.samples)
        self.ax.clear()
        plot_field(self.ax)
        self.plot_sample(self.sample_idx)
        self.fig.canvas.draw()

    def plot(self):
        """
        Setup the window and connect key_press events.
        """
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        plot_field(self.ax)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.plot_sample(-1) # start with the last one
        figManager = plt.get_current_fig_manager()
        figManager.window.showMaximized()
        plt.show()

    def plot_sample(self, sample_idx):
        """
        Plot sample data and title for given index.
        """
        sample_idx %= len(self.data.samples)
        sample = self.data.samples[sample_idx]
        self.tickplotter.plot_io(sample.input, sample.output, self.ax)
        action_result_str = protobuf_enum2str(self.data.samples[sample_idx].output, 'actionresult')
        ts = self.timestamp_to_str(sample.timestamp)
        plt.title(f'action={self.action_type_str} status={action_result_str} sample={1+sample_idx}/{len(self.data.samples)} time={ts}')

    def timestamp_to_str(self, timestamp):
        """
        Convert timestamp to string.
        """
        timestamp_str = datetime.fromtimestamp(timestamp.seconds + timestamp.nanos / 1e9).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        return timestamp_str

    def print_info(self):
        """
        Print some info to stdout about the action.
        """
        print(f'     filename: {self.filename}')
        action_type = self.data.type
        action_type_str = protobuf_enum2str(self.data, 'type')
        print(f'       action: {action_type_str} ({action_type})')
        action_result = self.data.samples[-1].output.actionresult
        action_result_str = protobuf_enum2str(self.data.samples[-1].output, 'actionresult')
        resultDetails = ''
        if self.data.dirty:
            resultDetails = ' (dirty)'
        if action_result_str == 'FAILED':
            reason = self.data.samples[-1].diagnostics.failureReason
            resultDetails += ' (failureReason: ' + reason + ')'
        print(f'       result: {action_result_str}{resultDetails}')
        print(f'     #samples: {len(self.data.samples)}')
        start_timestamp = datetime.fromtimestamp(self.data.samples[0].timestamp.seconds + self.data.samples[0].timestamp.nanos / 1e9)
        end_timestamp = datetime.fromtimestamp(self.data.samples[-1].timestamp.seconds + self.data.samples[-1].timestamp.nanos / 1e9)
        duration = (end_timestamp - start_timestamp).total_seconds()
        start_timestamp_str = start_timestamp.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        end_timestamp_str = end_timestamp.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        print(f'        start: {start_timestamp_str}')
        print(f'          end: {end_timestamp_str}')
        print(f'     duration: {duration:.3f} [s]')


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
