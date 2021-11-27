from LTP.Trajectory import MidTrajectory
from LTP.ComputeVelocities import compute_velocities
from LTP.TrackMap import TrackMap
from LTP.Utils import euclidean_distance_no_sqrt
from LTP.GraphicUtility import plot_track_map, plot_trajectory, end_plotting
from Stp import STP

race_track = "bigtrack2"
trackMap = TrackMap()
trackMap.load_track(f'./src/LTP/tests/tracks/{race_track}.json')
#create MidTrajectory object
trajectory = MidTrajectory(distance=euclidean_distance_no_sqrt)
#compute the trajectory
trajectory.compute_trajectory(trackMap)
#print the first plan step
print(trajectory.get_trajectory()[0])
trajectory.set_trajectory(compute_velocities(trajectory.get_trajectory()))
print(trajectory.trajectory[0])

s = {
    "track":        ((125, 329), (0.1, 5)),
    "track2":       ((233, 200), (0.1, -5)),
    "track3":       ((243, 455), (0.1, 5)),
    "bigtrack":     ((250, 468), (-2, 5)),
    "bigtrack2":    ((238, 87), (-3, 5)),
    "TarascoRace":  ((220, 303), (0.1, -5))
}

car_start_pos, car_start_vel = s[race_track][0], s[race_track][1]

trackMap.set_car_position(car_start_pos)
plot_track_map(trackMap, new_figure=True)
plot_trajectory(trajectory.get_trajectory(), new_figure=False)
end_plotting()

stp = STP()
stp.set_trajectory(trajectory)
stp.set_car_pos_vel(car_start_pos, car_start_vel)

i=0
while True:
    print("-"*20 + f" Step: {i}" + "-"*20)

    new_car_x, new_car_y, rdx, rdy, plan_ref = stp.compute()
    trackMap.set_prev_car_position(trackMap.get_car_position())
    trackMap.set_car_position((new_car_x,new_car_y))
    trackMap.set_ref(plan_ref)
    trackMap.set_prev_car_speed(stp.car.get_speed())
    trackMap.set_car_speed((rdx, rdy))
    plot_track_map(trackMap, new_figure=False)
    plot_trajectory(trajectory.get_trajectory(), new_figure=False)
    end_plotting()
    stp.set_car_pos_vel((new_car_x,new_car_y),(rdx,rdy))
    i += 1