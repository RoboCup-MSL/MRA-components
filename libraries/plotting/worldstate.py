import libraries.plotting


def plot(ax, worldstate):
    result = []
    result += libraries.plotting.plot_robot(ax, worldstate.robot, color='b')
    if worldstate.HasField('ball'):
        result += libraries.plotting.plot_ball(ax, worldstate.ball)
    for teammate in worldstate.teammates:
        result += libraries.plotting.plot_robot(ax, teammate, color='c')
    for opponent in worldstate.opponents:
        result += libraries.plotting.plot_robot(ax, opponent, color='r')
    for obstacle in worldstate.obstacles:
        result += libraries.plotting.plot_obstacle(ax, obstacle)
    return result

