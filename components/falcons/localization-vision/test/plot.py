#!/usr/bin/env python3

'''
Plot the diagnostics field in Local as serialized opencv object.
'''

# python modules
import sys, os
import argparse
import json
import numpy as np
import cv2
import struct
from matplotlib import pyplot as plt

# protobuf imports
SCRIPT_FOLDER = os.path.dirname(os.path.realpath(__file__))
os.chdir(SCRIPT_FOLDER)
MRA_BASE_FOLDER = os.path.realpath(os.path.join(SCRIPT_FOLDER, '../../../..'))
sys.path.append(f'{MRA_BASE_FOLDER}/bazel-bin')
from datatypes import CvMat_pb2
try:
    import Local_pb2
except:
    os.system(f'protoc -I={MRA_BASE_FOLDER} -I={MRA_BASE_FOLDER}/datatypes -I=../interface --python_out=. ../interface/Local.proto')
    import Local_pb2



def parse_args(args: list) -> argparse.Namespace:
    """
    Use argparse to parse command line arguments.
    """
    descriptionTxt = __doc__
    exampleTxt = ""
    class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass
    parser = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=CustomFormatter)
    parser.add_argument('-c', '--cvmatproto', help='treat given file as CvMatProto object', action='store_true')
    parser.add_argument('datafile', help='data file to load')
    return parser.parse_args(args)


def load_image_from_tick_bin(filename):
    # data is serialized, see MRA libraries/logging/logging.hpp dumpToFile
    # each protobuf object is an int (#bytes) followed by serialized protobuf bytes
    # dumped data:
    #    input
    #    params
    #    state (before)
    #    output
    #    local
    #    state (after)
    data = None
    with open(filename, "rb") as f:
        for it in range(5):
            # read next
            n = struct.unpack('i', f.read(4))[0]
            data = f.read(n)
    local = Local_pb2.Local()
    local.ParseFromString(data)
    image = local.floor
    print(f'{image.height} x {image.width}')
    return image


def load_image_from_cvmatproto_bin(filename):
    data = None
    with open(filename, "rb") as f:
        data = f.read()
    image = CvMat_pb2.CvMatProto()
    image.ParseFromString(data)
    print(f'{image.height} x {image.width}')
    return image


def main(args: argparse.Namespace) -> None:
    """
    Make the plot.
    """
    if args.cvmatproto:
        image = load_image_from_cvmatproto_bin(args.datafile)
    else:
        image = load_image_from_tick_bin(args.datafile)

    np_data = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)

    plt.imshow(cv2.cvtColor(np_data, cv2.COLOR_BGR2RGB))
    plt.axis("off")
    plt.show()


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))
