#!/usr/bin/env python3

"""
Control, configure or get status of the MRA logger.

Main use cases:
    show        show current configuration
    json        show current configuration as json string
    reset       reset current configuration
    enable      modify configuration: enable logging
    disable     modify configuration: disable logging
    wipe        wipe log folder (intended for test mode)
    list        list contents of log folder

For further customization of the configuration, just modify the json string in the configuration file directly.
"""

EXAMPLE_TEXT = """Examples:
* configure logging of a single component
  $ ./MRA-logger.py enable --scope RobotsportsGetBallFetch
* enable logging and tracing from test suite (as called from MRA-build.py)
  $ ./MRA-logger.py enable --test --tracing
"""


# python modules
import os, sys
import subprocess
import json, yaml
import argparse


LOGGER_SHM_FILES = {
    'default': '/dev/shm/mra_logging_shared_memory',
    'test': '/dev/shm/testsuite_mra_logging_shared_memory',
}
LOGGER_FOLDERS = {
    'default': '/tmp/mra_logging',
    'test': '/tmp/testsuite_mra_logging',
}
MODES = ['show', 'json', 'reset', 'enable', 'disable', 'wipe', 'list']
CONTEXTS = ['default', 'test']
LEVELS = ['CRITICAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG', 'TRACE']



def parse_args(args: list) -> argparse.Namespace:
    """Use argparse to parse command line arguments."""
    descriptionTxt = __doc__
    exampleTxt = EXAMPLE_TEXT
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        def __init__(self, prog):
            super().__init__(prog, width=100, max_help_position=30)
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('-s', '--scope', help='apply to custom component scope, comma-separated lists of (brief) component names', type=str)
    parser.add_argument('-t', '--test', help='test mode, intended for testsuite, data gets thrown away each time', action='store_true')
    parser.add_argument('-T', '--tracing', help='enable/keep tracing during tests', action='store_true')
    parser.add_argument('-b', '--tickbins', help='dump binary files, one per component tick', action='store_true')
    parser.add_argument('-n', '--dryrun', help='only print commands', action='store_true')
    parser.add_argument('mode', help='control mode', choices=MODES)
    return parser.parse_args(args)


class Configurator():
    def __init__(self, args):
        self.args = args
        self.context = CONTEXTS[args.test]
        self.level = ['INFO', 'TRACE'][self.args.tracing]
        assert(self.level in LEVELS)
        assert(self.context in CONTEXTS)
        self.config_filename = LOGGER_SHM_FILES[self.context]
        self.log_folder = LOGGER_FOLDERS[self.context]
        self.enabled = True
        self.dump_ticks = self.args.tickbins
        self.config_data = None

    def make_subconfig(self, component_name):
        return {
            'component': component_name,
            'level': self.level,
            'enabled': self.enabled,
            'dumpTicks': self.dump_ticks,
            'maxLineSize': 1000,
            'maxFileSizeMB': 10,
            'pattern': '[%Y-%m-%dT%H:%M:%S.%f] [%P/%t/%k] [%^%l%$] [%s:%#,%!] %v',
        }

    def make_config(self):
        result = {
            'folder': self.log_folder,
            'filename': '<maincomponent>_<pid>.spdlog',
            'general': self.make_subconfig('MRA'),
            'overrules': [],
        }
        return result

    def get_subconfig(self, component_name):
        for o in self.config_data['overrules']:
            if o['component'] == component_name:
                return o
        new_subconfig = self.make_subconfig(component_name)
        self.config_data['overrules'].append(new_subconfig)
        return new_subconfig

    def command(self, cmd):
        if self.args.dryrun:
            print(cmd)
        else:
            subprocess.check_call(cmd, shell=True)

    def reset(self):
        self.config_data = self.make_config()
        self.write()

    def read(self):
        if not os.path.isfile(self.config_filename):
            self.reset()
        self.config_data = yaml.safe_load(open(self.config_filename))

    def set(self, key, value):
        if self.args.scope:
            for component in self.args.scope.split(','):
                cfg = self.get_subconfig(component)
                cfg[key] = value
        else:
            self.config_data['general'][key] = value

    def write(self):
        s = json.dumps(self.config_data)
        self.command(f'echo \'{s}\' > {self.config_filename}')

    def display(self):
        print(f'current configuration (from {self.config_filename}):')
        print(self.config_data)

    def wipe(self):
        self.command(f'rm -rf {self.log_folder}')

    def list(self):
        print(f'contents of log folder {self.log_folder}:')
        self.command(f'ls -altr1 {self.log_folder}')

    def run(self):
        mode = self.args.mode
        self.read()
        self.set('dumpTicks', self.args.tickbins)
        if mode == 'show':
            self.display()
        elif mode == 'json':
            self.command(f'cat {self.config_filename}')
        elif mode == 'reset':
            self.reset()
        elif mode == 'enable':
            self.set('enabled', True)
            self.write()
        elif mode == 'disable':
            self.set('enabled', False)
            self.write()
        elif mode == 'wipe':
            self.wipe()
        elif mode == 'list':
            self.list()


def main(args) -> None:
    """Perform the work."""
    c = Configurator(args)
    c.run()


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

