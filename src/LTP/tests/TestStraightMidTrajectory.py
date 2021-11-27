import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Trajectory import MidTrajectory
from GraphicUtility import plot_track_map, plot_trajectory, end_plotting
from ComputeVelocities import compute_velocities
from TrackMap import TrackMap
from Utils import serialize_to_file, euclidean_distance_no_sqrt
from RiskFunctions import risk_laps, constant
from CarConstants import MIN_RISK, MAX_RISK
import time
from SampleTrack import StraightTrackMap

NAME_TEST = "TestStraight"


straight_track_map = StraightTrackMap(100, 10, 25, 5, 10)
straight_track_map.set_car_position((10, 10))

trajectory = MidTrajectory(distance_fun=euclidean_distance_no_sqrt)
trajectory.set_risk(constant(1, MIN_RISK, MAX_RISK))
print(trajectory.current_risk)
trajectory.update_max_velocity()
print(trajectory.current_max_velocity)
trajectory.compute_trajectory(straight_track_map)
trajectory.compute_velocities()

plot_track_map(straight_track_map, new_figure=True)
plot_trajectory(trajectory.get_trajectory(), new_figure=False)
end_plotting()