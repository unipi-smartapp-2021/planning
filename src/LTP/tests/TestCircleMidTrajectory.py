import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SampleTrack import CircleTrackMap
from Trajectory import MidTrajectory
from GraphicUtility import plot_track_map, plot_trajectory, end_plotting
from ComputeVelocities import compute_velocities
from Utils import serialize_to_file
from TrajectoryOptimizer import TrajectoryOptimizer

circle_track_map = CircleTrackMap(100, 10, 25, 5, 10)
circle_track_map.set_car_position((10, 10))

trajectory = MidTrajectory()
trajectory.compute_trajectory(circle_track_map)

final_trajectory = compute_velocities(trajectory.get_trajectory())

plot_track_map(circle_track_map, new_figure=True)
plot_trajectory(final_trajectory, new_figure=False)
   
end_plotting()