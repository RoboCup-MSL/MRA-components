import libraries.plotting


def plot(ax, worldstate):
    libraries.plotting.plot_robot(ax, worldstate.robot, color='b')
    if worldstate.HasField('ball'):
        libraries.plotting.plot_ball(ax, worldstate.ball)
    for teammate in worldstate.teammates:
        libraries.plotting.plot_robot(ax, teammate, color='c')
    for opponent in worldstate.opponents:
        libraries.plotting.plot_robot(ax, opponent, color='r')
    for obstacle in worldstate.obstacles:
        libraries.plotting.plot_obstacle(ax, obstacle)
