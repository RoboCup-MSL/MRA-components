import numpy as np
from matplotlib.patches import Ellipse, Rectangle, Circle


def plot(ax):
    # NOTE: x and y are inverted because we like to see the field in 'landscape' mode, not 'portrait'
    line_thickness = 0.12

    field_width = 12.0
    field_length = 18.0
    penalty_area_width = 6.5
    penalty_area_length = 2.25
    goal_area_width = 3.5
    goal_area_length = 0.75
    goal_width = 2.0
    goal_depth = 0.3

    # field
    ax.add_patch(Rectangle((-field_length/2,-field_width/2), field_length, line_thickness, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((-field_length/2,field_width/2), field_length, -line_thickness, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((-field_length/2,-field_width/2), line_thickness, field_width, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((field_length/2,-field_width/2), -line_thickness, field_width, fill=True, color='w', zorder=-1))

    # center
    ax.add_patch(Rectangle((-line_thickness/2,-field_width/2), line_thickness, field_width, fill=True, color='w', zorder=-1))

    # goal
    ax.add_patch(Rectangle((field_length/2,-goal_width/2), goal_depth, goal_width, fill=True, color='w', zorder=-1))
    # ax.add_patch(Rectangle((field_length/2 - line_thickness,-goal_width/2), goal_depth+line_thickness, goal_width, fill=False))

    ax.add_patch(Rectangle((-field_length/2 - goal_depth,-goal_width/2), goal_depth, goal_width, fill=True, color='w', zorder=-1))
    # ax.add_patch(Rectangle((-field_length/2 - goal_depth,-goal_width/2), goal_depth+line_thickness, goal_width, fill=False))

    # goal area
    ax.add_patch(Rectangle((field_length/2 - goal_area_length,-goal_area_width/2), line_thickness, goal_area_width, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((field_length/2,-goal_area_width/2), -goal_area_length, line_thickness, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((field_length/2,goal_area_width/2), -goal_area_length, -line_thickness, fill=True, color='w', zorder=-1))

    ax.add_patch(Rectangle((-field_length/2 + goal_area_length,-goal_area_width/2), -line_thickness, goal_area_width, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((-field_length/2,-goal_area_width/2), goal_area_length, line_thickness, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((-field_length/2,goal_area_width/2), goal_area_length, -line_thickness, fill=True, color='w', zorder=-1))

    # penalty area
    ax.add_patch(Rectangle((field_length/2 - penalty_area_length,-penalty_area_width/2), line_thickness, penalty_area_width, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((field_length/2,-penalty_area_width/2), -penalty_area_length, line_thickness, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((field_length/2,penalty_area_width/2), -penalty_area_length, -line_thickness, fill=True, color='w', zorder=-1))

    ax.add_patch(Rectangle((-field_length/2 + penalty_area_length,-penalty_area_width/2), -line_thickness, penalty_area_width, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((-field_length/2,-penalty_area_width/2), penalty_area_length, line_thickness, fill=True, color='w', zorder=-1))
    ax.add_patch(Rectangle((-field_length/2,penalty_area_width/2), penalty_area_length, -line_thickness, fill=True, color='w', zorder=-1))

    # center circle
    # TODO: properly add thickness to the circle
    ax.add_patch(Circle((0.0,0.0), 2.0, lw=1.5*line_thickness/0.0352777778, fill=False, color='w', zorder=-1))

    ax.set_xticks(np.arange(-20,20,2))
    ax.set_yticks(np.arange(-20,20,2))
    ax.set_yticklabels([str(v) for v in range(20, -20, -2)]) # invert axis labels ...
    # TODO: a better way to rotate the plot to align with our FCS might be to rotate entire axis, see
    # https://stackoverflow.com/questions/21652631/how-to-rotate-a-simple-matplotlib-axes

    ax.set_ylim(-7, 7)
    ax.set_xlim(-10, 10)

    # make field green
    ax.set_facecolor((80.0/250.0,208.0/250.0,128.0/250.0))

    ax.grid(True, zorder=0)
    ax.set_aspect('equal')
