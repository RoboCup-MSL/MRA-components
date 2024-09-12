#!/usr/bin/env python3

"""
Data class to handle tick .bin files. See also: MRA-print.py.
"""


# python modules
import os
import sys
#os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import struct
import importlib

# this datatype is always needed
# the others depend on the component specified in the header, those are dynamically imported
try:
    import datatypes.Meta_pb2
except ImportError:
    # try cmake build/folder
    # it works for now (for MRA-tickbin.py), but TODO: make this nicer ...
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'build'))
    try:
        import datatypes.Meta_pb2
    except ImportError:
        raise ImportError('failed to import datatypes.Meta_pb2, perhaps cmake build was not done?')
        # or try bazel build location?



# constants
FILE_ELEMENTS = (
    'meta',
    'input',
    'params',
    'state_before',
    'output',
    'diagnostics',
    'state_after',
    )
EXPECTED_FILE_ELEMENT_COUNT = len(FILE_ELEMENTS)
# these are all protobuf objects



class Data():

    def __init__(self, filename):
        self.load(filename)

    def load(self, filename):
        # load all binary data, uninterpreted
        self.raw_load(filename)
        # parse meta header to find out the type(s) to use when interpreting raw data
        self.parse_meta_header()
        # assign raw_data to types of the current component
        self.parse_component_data()

    def raw_load(self, filename):
        self.raw_data = {}
        loaded_elem_count = 0
        with open(filename, 'rb') as f:
            for key in FILE_ELEMENTS:
                n = struct.unpack('i', f.read(4))[0]
                self.raw_data[key] = f.read(n)
                loaded_elem_count += 1
        if loaded_elem_count != EXPECTED_FILE_ELEMENT_COUNT:
            print('WARNING: file has {:d} elements, but {:d} were expected'.format(loaded_elem_count, EXPECTED_FILE_ELEMENT_COUNT))

    def parse_meta_header(self):
        self.meta = datatypes.Meta_pb2.Meta()
        self.meta.MergeFromString(self.raw_data['meta'])

    def parse_component_data(self):
        for elem in FILE_ELEMENTS[1:]: # skip first element (meta)
            key = elem.split('_')[0]
            Key = key.capitalize() # example Input
            # dynamic import of component interface
            key_module = '{:s}_pb2'.format(Key) # example Input_pb2
            module_name = 'components.{:s}.interface.{:s}'.format(self.meta.subfolder.replace('/', '.'), key_module, Key)
            try:
                module = importlib.import_module(module_name)
            except:
                continue
            # construct message and merge from raw data
            msg = getattr(module, Key)()
            msg.MergeFromString(self.raw_data[elem])
            # store
            setattr(self, elem, msg)



