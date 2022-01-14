from matplotlib.pyplot import plot
from RandomTracks import generate_random_track
from GraphicUtility import plot_track_map, plot_trajectory, end_plotting
from Parameters import Parameters
from Trajectory import Trajectory
import numpy as np
import time

ESTERNO = 0
IN_MEZZO = 1
INTERNO = 2

def generate_sample():
    track_map = generate_random_track()
    parameters = Parameters()
    trajectory = Trajectory(parameters)
    trajectory.compute_middle_trajectory(track_map)
    trajectory.compute_velocities()

    opt_trajectory = Trajectory(parameters)
    target = opt_trajectory.compute_optimal_trajectory(track_map)
    opt_trajectory.compute_velocities()

    # Per ogni punto della traiettoria ottimale avrò una lista di curvature e target (0, 1, 2)
    # poi calcoliamo anche il displacement dal punto precedente, l'ultimo da quanto si sposta dal penultimo,
    # il secondo da quanto si sposta dal primo, e così via. Il primo punto non ha displacement.
    curvatures = list(map(lambda x: x.curvature, trajectory.get_trajectory()))
    displacemets = [(0,0)] + list(map(lambda x1, x2: (x2.position[0] - x1.position[0], x2.position[1] - x1.position[1]) , trajectory.get_trajectory()[:-1], trajectory.get_trajectory()[1:]))

    trainingset = []
    for i in range(len(target)):
        trainingset.append([curvatures[i], *displacemets[i], target[i]])
    return trainingset

start = time.time()
N_SAMPLES = 1000
n_feature = 4
data = []
for i in range(N_SAMPLES):
    sample = np.array(generate_sample())
    # Concat sample in data
    data.append(sample)

# Concatenate all samples in one array
data = np.concatenate(data)

# Save data to file
np.savetxt("data.csv", data, delimiter=",")
end = time.time()
print("Time: ", end - start)