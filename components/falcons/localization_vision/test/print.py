#!/usr/bin/env python3

"""
Print data from .bin file. Can print all except CvMat objects, those can be plotted by plot.py.
"""


# python modules
import sys
import argparse
import json
from google.protobuf.message import Message
from google.protobuf.descriptor import FieldDescriptor
from google.protobuf.json_format import MessageToJson

# own modules
import common



BLACKLIST = []



def parse_args(args: list) -> argparse.Namespace:
    """
    Use argparse to parse command line arguments.
    """
    descriptionTxt = __doc__
    exampleTxt = ''
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('-l', '--landmarks', help='also print landmarks array', action='store_true')
    parser.add_argument('-J', '--no-json', dest='json', help='do not print as json string, instead use protobuf string formatting, multiline', action='store_false')
    parser.add_argument('-i', '--indent-json', help='json indentation to use', default=None)
    parser.add_argument('-b', '--binary', help='do not strip binary data', action='store_true')
    parser.add_argument('datafile', help='data file (.bin) to load')
    return parser.parse_args(args)


def reduce_protobuf_message(message: Message, options: dict = {}) -> Message:
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
            reduce_protobuf_message(getattr(message, field.name), options)
    return message


def main(args: argparse.Namespace) -> None:
    """
    Print the information.
    """
    # load the data
    data = common.Data(args.datafile)
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
    # start printing after optionally massaging data
    for e in common.FILE_ELEMENTS: # input, params, state, output etc
        # general reduction
        v = reduce_protobuf_message(getattr(data, e), options)
        # more custom reduction
        if e == 'input' and not args.landmarks:
            v.landmarks.clear()
        # print
        print('\n' + e + ':\n')
        if args.json:
            print(MessageToJson(v, indent=args.indent_json))
        else:
            print(v)


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

