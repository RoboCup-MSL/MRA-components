#!/usr/bin/env python3

"""
Print contents of a tick .bin file.
"""

EXAMPLE_TEXT = """Example:

    ./MRA-tickbin.py /tmp/testsuite_mra_logging/tickbins/tick_RobotsportsProveIsAlive_1.bin 

    meta:
    {"component": "RobotsportsProveIsAlive", "subfolder": "robotsports/prove_is_alive", "counter": 1, "timestamp": "2024-05-07T19:28:41.346076942Z", "duration": 0.0009431870000000001}

    input:
    {"worldstate": {"robot": {"active": true}}}

    params:
    {"angleInDegrees": 15.0, "angleToleranceDeg": 1.0, "maxTimePerPhase": 10.0}

    output:
    {"actionresult": "RUNNING", "target": {"position": {"rz": 0.2617993877991494}}}

    diagnostics:
    {}
"""


# python modules
import sys
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import argparse
from google.protobuf.message import Message
from google.protobuf.descriptor import FieldDescriptor
from google.protobuf.json_format import MessageToJson

# MRA tickbin Data class
from libraries.logging.logdata import Data, FILE_ELEMENTS



def parse_args(args: list) -> argparse.Namespace:
    """
    Use argparse to parse command line arguments.
    """
    descriptionTxt = __doc__
    exampleTxt = EXAMPLE_TEXT
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('-J', '--no-json', dest='json', help='do not print as json string, instead use protobuf string formatting, multiline', action='store_false')
    parser.add_argument('-i', '--indent-json', help='json indentation to use', default=None)
    parser.add_argument('-b', '--binary', help='do not strip binary data', action='store_true')
    parser.add_argument('-s', '--state', help='also print state, both before and after, instead of hiding', action='store_true')
    parser.add_argument('datafile', help='data file (.bin) to load')
    return parser.parse_args(args)


def protobuf_message_reduction(message: Message, options: dict = {}) -> Message:
    """
    Recursively removes / replaces data from a Protobuf message (protobuf v3).
    """
    # options
    if not 'binary_replacement' in options:
        options['binary_replacement'] = b''
    if not 'repeated_clear' in options:
        options['repeated_clear'] = False
    # modify
    for field in message.DESCRIPTOR.fields:
        if field.name in message.DESCRIPTOR.fields_by_name and message.DESCRIPTOR.fields_by_name[field.name].type == FieldDescriptor.TYPE_BYTES:
            if not options.get('binary_replacement') is None:
                setattr(message, field.name, options.get('binary_replacement'))
        elif field.name in message.DESCRIPTOR.fields_by_name and message.DESCRIPTOR.fields_by_name[field.name].label == FieldDescriptor.LABEL_REPEATED:
            if options.get('repeated_clear'):
                getattr(message, field.name).clear()
        elif isinstance(getattr(message, field.name), Message):
            protobuf_message_reduction(getattr(message, field.name), options)
    return message


def main(args: argparse.Namespace) -> None:
    """
    Print the information.
    """
    # load the data
    data = Data(args.datafile)
    # option handling
    options = {'repeated_clear': False, 'binary_replacement': b''}
    if args.binary: # not sure why this would be useful ...
        options['binary_replacement'] = None
    if args.indent_json:
        args.json = True
        # use ints if possible, the json formatter otherwise uses digits as prefix string
        try:
            args.indent_json = int(args.indent_json)
        except:
            pass
    # state before and after
    # TODO: option to calculate state change? difflib?
    if not args.state:
        for s in ('state_before', 'state_after'):
            if hasattr(data, s):
                delattr(data, s)
    # start printing after optionally massaging data
    elements = FILE_ELEMENTS # input, params, state, output etc
    for e in elements:
        if hasattr(data, e):
            # general reduction
            v = protobuf_message_reduction(getattr(data, e), options)
            # print
            print('\n' + e + ':\n')
            if args.json:
                print(MessageToJson(v, indent=args.indent_json))
            else:
                print(v)


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))


