"""
    Contains functions to plot the trajectory or the track
"""

from typing import List
import matplotlib.pyplot as plt
from LTP.PlanStep import PlanStep

def plot_line(x1, y1, x2, y2):
    """[summary]

    Args:
        x1 ([type]): [description]
        y1 ([type]): [description]
        x2 ([type]): [description]
        y2 ([type]): [description]
    """
    plt.plot([x1, x2], [y1, y2])

def plot_track_map(track_map, new_figure=True):
    """[summary]

    Args:
        track_map ([type]): [description]
        new_figure (bool, optional): [description]. Defaults to True.
    """
    if new_figure:
        plt.figure()
    plt.title('Track Map')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.scatter([x for x, _ in track_map.left_cones], [y for _, y in track_map.left_cones], color='blue', label='Left Cones')
    plt.scatter([x for x, _ in track_map.right_cones], [y for _, y in track_map.right_cones], color='yellow', label='Right Cones')
    # Plot starting point
    plt.scatter(track_map.left_cones[0][0], track_map.left_cones[0][1], color='black')
    plt.scatter(track_map.right_cones[0][0], track_map.right_cones[0][1], color='black')
    
    plot_line(track_map.left_cones[0][0], track_map.left_cones[0][1], track_map.left_cones[1][0], track_map.left_cones[1][1])
    plot_line(track_map.right_cones[0][0], track_map.right_cones[0][1], track_map.right_cones[1][0], track_map.right_cones[1][1])

    if track_map.car_position is not None:
        plt.scatter(track_map.car_position[0], track_map.car_position[1], color='red', label='Car Position')
    plt.draw()

def plot_trajectory(trajectory: List[PlanStep], new_figure=False):
    """[summary]

    Args:
        trajectory (List[PlanStep]): [description]
        new_figure (bool, optional): [description]. Defaults to False.
    """
    if new_figure:
        plt.figure()
    plt.title('Trajectory')
    plt.xlabel('x')
    plt.ylabel('y')
    velocities = list(map(lambda x: x*3.6, [plan_step.velocity for plan_step in trajectory]))
    # Plot the anchors
    plt.scatter([plan_step.position[0] for plan_step in trajectory], [plan_step.position[1] for plan_step in trajectory], c=velocities, label='velocity (km/h)')
    plt.colorbar()
    #plt.scatter([p.position[0] for p in trajectory if p.anchor], [p.position[1] for p in trajectory if p.anchor], color='red', label='anchor')
    plt.legend()
    plt.draw()

def end_plotting():
    """[summary]
    """
    plt.show()
