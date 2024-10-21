import numpy as np
from matplotlib.patches import Wedge
from libraries.plotting.transform import transform


ROBOT_RADIUS = 0.25
ROBOT_MOUTH_ANGLE = 50 # degrees


def plot(ax, robot, color='b'):
    # visualize robot as pacman, to show its orientation
    p = transform(robot.position)
    orientation = p[2]
    start_angle = np.degrees(orientation) + ROBOT_MOUTH_ANGLE * 0.5
    end_angle = np.degrees(orientation) - ROBOT_MOUTH_ANGLE * 0.5
    pacman = Wedge((p[0], p[1]), ROBOT_RADIUS, start_angle, end_angle, facecolor=color, edgecolor='black', linewidth=0.5, zorder=3.2)
    ax.add_patch(pacman)
