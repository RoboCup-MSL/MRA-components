#!/usr/bin/env python3

'''
View MRA tracing files.
'''

EXAMPLE_TXT = '''Example:
* after running the test suite, the following files may appear
    TODO
* running this script will automatically open and visualize them:
    MRA-traceview.py /tmp/testsuite_mra_logging
'''

# python modules
import os, sys
import subprocess
import glob

# get repo extendedlogging
try:
    import ttvlib
except ImportError:
    # try environment variable
    epath = os.getenv('PATH_TO_EXTENDEDLOGGING')
    if not epath:
        # try bazel dependency
        epath = os.path.join(subprocess.getoutput('bazel info output_base'), 'external', 'extendedlogging')
    if not os.path.isdir(epath):
        raise Exception('failed to find repo extendedlogging')
    sys.path.append(epath)
    import ttvlib

# where to look for the data
DEFAULT_FOLDER = '/tmp/mra_logging'



def parse_args():
    # modify cli
    parser = ttvlib.ttviewer.make_parser(__doc__, EXAMPLE_TXT)
    parser.add_argument('folder', help='folder to analyze', nargs='?', default=DEFAULT_FOLDER)
    return parser.parse_args()


def determine_filenames(folder):
    fn = glob.glob(os.path.join(folder, '*'))
    fn = [f for f in fn if os.path.isfile(f)]
    return fn


def pid_tid_handler(filename, jsondata):
    for j in jsondata:
        j['pid'] = os.path.basename(filename)


def main(**kwargs):
    # determine MRA tracing/logging files
    filenames = determine_filenames(kwargs.get('folder'))
    del kwargs['folder']
    # feed filenames and run the viewer
    kwargs['filenames'] = filenames
    kwargs['pid_tid_handler'] = pid_tid_handler
    ttvlib.ttviewer.run(**kwargs)


if __name__ == '__main__':
    # parse arguments, reuse ttviewer commandline interface
    kwargs = vars(parse_args())
    main(**kwargs)

