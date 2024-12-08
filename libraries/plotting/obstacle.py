from matplotlib.patches import Circle


def plot(ax, obstacle, color='k', size=0.25):
    ax.add_patch(Circle((obstacle.position.x, obstacle.position.y), size, fill=True, color=color, zorder=3.1))
