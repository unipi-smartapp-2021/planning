import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SampleTrack import CircleTrackMap
from Trajectory import Trajectory
from Parameters import Parameters
from GraphicUtility import plot_track_map, plot_trajectory, end_plotting
from PlanStep import PlanStep
import random
from TrackMap import TrackMap

random.seed(1)

circle_track_map = CircleTrackMap(50, 3, 10, 0, 0, 0, 180, swap_cones=True)

parameters = Parameters()
trajectory = Trajectory(parameters)
trajectory.compute_middle_trajectory(circle_track_map)
trajectory.compute_velocities()

print("initial trajectory")
plot_track_map(circle_track_map, new_figure=True)
plot_trajectory(trajectory.get_trajectory(), new_figure=False)


print("After forcing..")
trajectory.set_trajectory(trajectory.force_inside_track(circle_track_map))

plot_track_map(circle_track_map, new_figure=True)
plot_trajectory(trajectory.get_trajectory(), new_figure=False)
end_plotting()
