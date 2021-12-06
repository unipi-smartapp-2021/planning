from LTP.Trajectory import Trajectory
from LTP.GraphicUtility import plot_track_map, plot_trajectory, end_plotting
from LTP.TrackMap import TrackMap
from LTP.Utils import serialize_to_file, euclidean_distance_no_sqrt
from LTP.RiskFunctions import risk_laps

from LTP.Parameters import Parameters
from LTP.RaceState import RaceState
import time

NAME_TEST = "TestCustomTrack1"

start_time = time.time()

custom_track = TrackMap()
custom_track.load_track('src/LTP/tests/tracks/TarascoRace.json')

custom_track.set_car_position((298, 235))

#main LTP

parameters = Parameters()
raceState = RaceState()

#Race

trajectory = Trajectory(parameters, distance_fun=euclidean_distance_no_sqrt)

# Note: the risk has to be computed by the Race class since it depends also on the race type.
trajectory.set_risk(risk_laps(raceState.get_current_lap(), parameters.get_num_laps(), parameters.get_min_risk(), parameters.get_max_risk()))

trajectory.compute_middle_trajectory(custom_track)

trajectory.compute_velocities()

print("--- %s seconds ---" % (time.time() - start_time))

plot_track_map(custom_track, new_figure=True)
plot_trajectory(trajectory.get_trajectory(), new_figure=False)

serialize_to_file(custom_track.get_left_cones(), custom_track.get_right_cones(), trajectory.get_trajectory(), "./tests/output/" + NAME_TEST)

end_plotting()