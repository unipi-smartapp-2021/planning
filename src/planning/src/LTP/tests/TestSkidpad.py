import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Trajectory import Trajectory
from SampleTrack import skidpad
from GraphicUtility import plot_track_map, end_plotting, plot_trajectory
import matplotlib.pyplot as plt
from Parameters import Parameters

list_track_map = skidpad(20, 15.25/2, 20, [10, 20, 10], 3)

parameters = Parameters()
for track_map in list_track_map:
    trajectory = Trajectory(parameters)

    trajectory.compute_middle_trajectory(track_map)
    trajectory.compute_velocities()

    plot_track_map(track_map, new_figure=True)
    plot_trajectory(trajectory.get_trajectory(), new_figure=False)

    end_plotting()