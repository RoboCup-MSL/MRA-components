from libraries.plotting import plot_robot
from libraries.plotting.transform import transform


TARGET_ALPHA = 0.5


def plot(ax, target, robot):
    result = []
    # use robot visualization, with a dotted line connecting to current robot position
    result += plot_robot(ax, target, color='grey', alpha=TARGET_ALPHA)
    # draw a line from the robot to the target
    rp = transform(robot.position)
    tp = transform(target.position)
    result += ax.plot([rp[0], tp[0]], [rp[1], tp[1]], color='grey', linewidth=3, linestyle='dotted', zorder=3.3, alpha=TARGET_ALPHA)
    return result

