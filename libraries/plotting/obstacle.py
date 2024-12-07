from matplotlib.patches import Circle


def plot(ax, obstacle, color='k'):
    ax.add_patch(Circle((obstacle.x, obstacle.y), 0.1, fill=True, color=color, zorder=3.1))
