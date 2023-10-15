#!/usr/bin/env python3

"""
Plot data to diagnose what happens at a tick.

Inputs / modes:
1. binary file with only a CvMatProto object -> just plot it
2. binary tick data file -> then plot the diagnostics field (local.floor)

Example (demo1.png):
* run a specific test and with tick tracing enabled:
    ./MRA-build.py -t -T -s vision -- --test_arg=--gtest_filter=FalconsLocalizationVisionTest.jsonTest3GrabsR5BadInit
* check that the file appeared:
    /tmp/testsuite_mra_logging/tickbins/tick_FalconsLocalizationVision_0.bin
* plot the diagnostics image (local.floor)
    bazel run //components/falcons/localization_vision/test:plot /tmp/testsuite_mra_logging/tickbins/tick_FalconsLocalizationVision_0.bin
"""


# python modules
import sys, os
import argparse
import numpy as np
import cv2
from matplotlib import pyplot as plt

# own modules
import common




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
    data = common.Data(args.datafile)
    image = data.local.floor
    np_data = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    plt.imshow(cv2.cvtColor(np_data, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.title(f'{args.datafile} ({image.height} x {image.width})')
    plt.show()


if __name__ == '__main__':
    main(parse_args(sys.argv[1:]))

