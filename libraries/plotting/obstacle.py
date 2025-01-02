from matplotlib.patches import Circle
from libraries.plotting.transform import transform


def plot(ax, obstacle, color='k', size=0.25):
    p = transform(obstacle.position)
    obst = Circle((p[0], p[1]), size, fill=True, color=color, zorder=3.1)
    ax.add_patch(obst)
    return [obst]

