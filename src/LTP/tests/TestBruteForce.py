import os, sys
from turtle import title
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from matplotlib.pyplot import plot
from RandomTracks import generate_random_track
from GraphicUtility import plot_track_map, plot_trajectory, end_plotting
from TrackMap import TrackMap
import yaml
from SampleTrack import StraightTrackMap
from Trajectory import Trajectory
import copy
from Parameters import Parameters
from Utils import serialize_to_file

import numpy as np
import random
import matplotlib.pyplot as plt
import time

# Set seed
random.seed(23)
np.random.seed(23)

track_map = generate_random_track()

start = time.time()
parameters = Parameters()
trajectory = Trajectory(parameters)

trajectory.compute_middle_trajectory(track_map)
trajectory.compute_velocities()

print('time: ', trajectory.compute_time())

plot_track_map(track_map, title='Middle trajectory, time: ' + str(trajectory.compute_time()))
plot_trajectory(trajectory.get_trajectory())

end = time.time() - start

# Optimize trajectory via brute force

start_opt = time.time()
opt_trajectory = Trajectory(parameters)
target = opt_trajectory.compute_optimal_trajectory(track_map)
opt_trajectory.compute_velocities()

print('opt time: ', opt_trajectory.compute_time())

plot_track_map(track_map, new_figure=True, title='Optimal trajectory, time: ' + str(opt_trajectory.compute_time()))
plot_trajectory(opt_trajectory.get_trajectory())
end_opt = time.time() - start_opt

print('Time to compute middle trajectory: ', end)
print('Time to compute optimal trajectory: ', end_opt)
end_plotting()