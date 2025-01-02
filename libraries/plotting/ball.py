from matplotlib.patches import Circle
from libraries.plotting.transform import transform


BALL_RADIUS = 0.12


def plot(ax, ball, color='yellow'):
    p = transform(ball.position)
    b = Circle((p[0], p[1]), BALL_RADIUS, fill=True, edgecolor='black', facecolor=color, linewidth=0.5, zorder=3.5)
    ax.add_patch(b)
    # arrow to visualize the ball's velocity
    arrow_scaling = 0.6
    v = transform(ball.velocity)
    d = arrow_scaling * v
    a = ax.arrow(p[0], p[1], d[0], d[1], width=0.05, linewidth=0.3, fc=color, ec='black', zorder=3)
    return [b, a]

