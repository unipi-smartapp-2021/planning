from typing import List
from LTP.GraphicUtility import plot_trajectory
from LTP.TrackMap import TrackMap
from LTP.PlanStep import PlanStep
from LTP.ComputeVelocities import compute_velocities, compute_time
import random
from LTP.Utils import find_line, compute_spline, force_inside_track
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as si
import cma

def CMA_ES(track_map: TrackMap, initial_trajectory: List[PlanStep], N_steps: int, seed=42) -> List[PlanStep]:
    def cost(trajectory: List[float]) -> float:
        new_trajectory = [PlanStep((x, y)) for x, y in zip(trajectory[::2], trajectory[1::2])]
        return compute_time(compute_velocities(force_inside_track(track_map, new_trajectory)))
    # Flatten trajectory into list of points [x_1, y_1, x_2, y_2, ...]
    initial_points = [step.position for step in initial_trajectory]
    def foldl(f, acc, l):
        for x in l:
            acc = f(acc, x)
        return acc
    initial_points = foldl(lambda acc, x: acc + [x[0], x[1]], [], initial_points)
    xopt, es = cma.fmin2(cost, initial_points, 0.5, options={'popsize': len(initial_points)//2, 'maxiter': N_steps, 'seed': seed})
    print("Final time = ", compute_time(compute_velocities(force_inside_track(track_map, [PlanStep((x, y)) for x, y in zip(xopt[::2], xopt[1::2])]))))
    return [PlanStep((x, y)) for x, y in zip(xopt[::2], xopt[1::2])]


    """
    CMA-ES algorithm to find the best trajectory
    """
    def compute_mean(values: List[float]) -> float:
        return sum(values)/len(values)

    def compute_variance(values: List[float]) -> float:
        mean = compute_mean(values)
        return sum([(value - mean)**2 for value in values])/len(values)

    def compute_variance_xy(values_x: List[float], values_y) -> float:
        mean_x = compute_mean(values_x)
        mean_y = compute_mean(values_y)
        return sum([(value_x - mean_x)*(value_y - mean_y) for value_x, value_y in zip(values_x, values_y)])/len(values_x)

    def sample_trajectory(means_x, means_y, C_x, C_y, C_xy) -> List[PlanStep]:
        # Sample a trajectory from the gaussian distribution
        new_trajectory = []
        for i in range(0, len(means_x)):
            # Sample a point from the gaussian distribution
            x = random.gauss(means_x[i], C_x[i])
            y = random.gauss(means_y[i], C_y[i])
            # Sample from multivariate normal
            x = np.random.multivariate_normal(means_x, cov, 1)
            step = PlanStep((x, y))
            step.anchor = False
            new_trajectory.append(step)
        return new_trajectory
    random.seed(seed)
    # Define a N_anchors multivariate Normal distribution with mean 0 and covariance matrix C
    N_anchors = len(initial_trajectory)
    # Initiale the means and variances to the initial trajectory points
    means_x = [step.position[0] for step in initial_trajectory]
    means_y = [step.position[1] for step in initial_trajectory]
    C_x = [1]*N_anchors
    C_y = [1]*N_anchors
    C_xy = [1]*N_anchors
    population_size = 50
    N_best = int(0.25 * population_size)
    for t in range(N_steps):
        # Sample a population
        population = []
        for i in range(population_size):
            trajectory = sample_trajectory(means_x, means_y, C_x, C_y, C_xy)
            population.append(force_inside_track(track_map, trajectory))
        # Compute the fitness of the population
        fitness = []
        for i in range(population_size):
            fitness.append(compute_time(compute_velocities(population[i])))
        # Find the best N_best individuals
        best_indices = sorted(range(len(fitness)), key=lambda k: fitness[k])[:N_best]
        best_population = [population[i] for i in best_indices]
        # Update the means and variances
        for anchor in range(N_anchors):
            x_values = [trajectory[anchor].position[0] for trajectory in best_population]
            y_values = [trajectory[anchor].position[1] for trajectory in best_population]
            means_x[anchor] = compute_mean(x_values)
            means_y[anchor] = compute_mean(y_values)
            C_x[anchor] = compute_variance(x_values)
            C_y[anchor] = compute_variance(y_values)
            C_xy[anchor] = compute_variance_xy(x_values, y_values)
        print("Iteration: ", t, " Fitness: ", fitness[best_indices[0]])
    return best_population[0]

def interpolate(x_i, y_i, anchors, N_points=100):
    # Find the interpolation of the anchors
    x_i_anchors = [x_i[anchor] for anchor in anchors] + [x_i[anchors[0]]] # Add the first point to the end to close the curve
    y_i_anchors = [y_i[anchor] for anchor in anchors] + [y_i[anchors[0]]]
    n = len(x_i_anchors)
    plotpoints = N_points
    k = 5
    knotspace = range(n)
    knots = si.InterpolatedUnivariateSpline(knotspace, knotspace, k=k).get_knots()
    knots_full = np.concatenate(([knots[0]]*k, knots, [knots[-1]]*k))
    tckX = knots_full, x_i_anchors, k
    tckY = knots_full, y_i_anchors, k
    splineX = si.UnivariateSpline._from_tck(tckX)
    splineY = si.UnivariateSpline._from_tck(tckY)
    tP = np.linspace(knotspace[0], knotspace[-1], plotpoints)
    xP = splineX(tP)
    yP = splineY(tP)
    return xP, yP

def optimize_trajectory(track_map: TrackMap, trajectory: List[PlanStep], N_opt_steps=1000, seed=42) -> List[PlanStep]:
    random.seed(seed)
    # We start by taking the anchors near curves by looking at where the first or second derivative changes sign
    anchors = []
    _, x_i,y_i,dx_i,dy_i,ddx_i,ddy_i = compute_spline(trajectory)
    anchor_trajectory = []
    # Find the curves of the spline, we consider a curve when the derivative changes sign
    for i in range(0, len(trajectory)):
        if random.random() < 0.3 or dx_i[i]*dx_i[(i+1)%len(trajectory)] < 0 or dy_i[i]*dy_i[(i+1)%len(trajectory)] < 0 or ddx_i[i]*ddx_i[(i+1)%len(trajectory)] < 0 or ddy_i[i]*ddy_i[(i+1)%len(trajectory)] < 0:
            anchors.append(i)
            trajectory[i].anchor = True
            anchor_trajectory.append(trajectory[i])
        else:
            trajectory[i].anchor = False
    print("Anchors: ", anchors)
    # Find the interpolation of the anchors and sample multiple points
    x_i_anchors, y_i_anchors = interpolate(x_i, y_i, anchors, N_points=100)
    new_trajectory = [PlanStep((x, y)) for x, y in zip(x_i_anchors[:-1], y_i_anchors[:-1])] # Remove the last point since it's only replicated to close the loop
    print("MidTrajectory Time: ", compute_time(compute_velocities(force_inside_track(track_map, trajectory))))
    trajectory = CMA_ES(track_map, [step for step in new_trajectory], N_steps=N_opt_steps, seed=seed)
    for i in range(len(trajectory)):
        trajectory[i].anchor = False
    print("Optimized Time: ", compute_time(compute_velocities(force_inside_track(track_map, trajectory))))
    return trajectory

# # The idea of this function is to take some random points (called anchors) and modify
# # their position randomly to hopefully find a faster trajectory.
# def optimize_trajectory(track_map: TrackMap, trajectory: List[PlanStep], N_opt_steps=1000, seed=42) -> List[PlanStep]:
#     random.seed(seed)
#     # Take random anchors
#     anchors = []
#     _, x_i,y_i,dx_i,dy_i,ddx_i,ddy_i = compute_spline(trajectory)
#     # Find the curves of the spline, we consider a curve when the derivative changes sign
#     for i in range(0, len(trajectory)):
#         if dx_i[i]*dx_i[(i+1)%len(trajectory)] < 0 or dy_i[i]*dy_i[(i+1)%len(trajectory)] < 0 or ddx_i[i]*ddx_i[(i+1)%len(trajectory)] < 0 or ddy_i[i]*ddy_i[(i+1)%len(trajectory)] < 0:
#             anchors.append(i)
#             trajectory[i].anchor = True
#         else:
#             trajectory[i].anchor = False
#     print("Anchors: ", anchors)
#     # TODO: Refactor into compute_spline
#     # Find the interpolation of the anchors
#     x_i_anchors = [x_i[anchor] for anchor in anchors] + [x_i[anchors[0]]] # Add the first point to the end to close the curve
#     y_i_anchors = [y_i[anchor] for anchor in anchors] + [y_i[anchors[0]]]
#     n = len(x_i_anchors)
#     plotpoints = 100
#     k = 5
#     knotspace = range(n)
#     knots = si.InterpolatedUnivariateSpline(knotspace, knotspace, k=k).get_knots()
#     knots_full = np.concatenate(([knots[0]]*k, knots, [knots[-1]]*k))
#     tckX = knots_full, x_i_anchors, k
#     tckY = knots_full, y_i_anchors, k
#     splineX = si.UnivariateSpline._from_tck(tckX)
#     splineY = si.UnivariateSpline._from_tck(tckY)
#     tP = np.linspace(knotspace[0], knotspace[-1], plotpoints)
#     xP = splineX(tP)
#     yP = splineY(tP)
#     # ---- END INTERPOLATION --
#     best_time = compute_time(trajectory)
#     best_trajectory = list(trajectory)
#     print('best time:', best_time)
#     new_trajectory = [PlanStep((x, y)) for x, y in zip(xP[:-1], yP[:-1])]
#     new_trajectory = force_inside_track(track_map, new_trajectory)
#     new_trajectory = compute_velocities(new_trajectory)
#     new_time = compute_time(new_trajectory)
#     print('new time:', new_time)
#     # Set the anchors
#     for i in range(len(new_trajectory)):
#         if i in anchors:
#             new_trajectory[i].anchor = True
#         else:
#             new_trajectory[i].anchor = False
#     return new_trajectory

    
#     # Naive approach: modify the position of the anchors randomly
#     # The only constraint is that for each anchor we don't want to send it off track
#     # constraint = lambda x: -track_width/2 < x and x < track_width/2
#     best_time = compute_time(trajectory)
#     best_trajectory = list(trajectory)
#     print('best time:', best_time)
#     for i in range(0, N_opt_steps):
#         # Deep copy the trajectory
#         new_trajectory = [PlanStep(plan_step.position) for plan_step in trajectory]
#         for anchor in anchors:
#             # Find the closest cones
#             left_cone = track_map.get_nearest_left_cone(new_trajectory[anchor].position)
#             right_cone = track_map.get_nearest_right_cone(new_trajectory[anchor].position)
#             # Find the line connecting the two cones
#             m, q = find_line(left_cone, right_cone)
#             # Take a random point on the line m, q
#             x = random.uniform(min(left_cone[0], right_cone[0]), max(left_cone[0], right_cone[0]))
#             #x = min(left_cone[0], right_cone[0]) + np.random.normal(0, compute_distance(left_cone, right_cone)/2)
#             y = m * x + q
#             # Modify the position of the anchor
#             new_trajectory[anchor].position = x, y
#         # Recompute the velocities of the new_trajectory
#         new_trajectory = compute_velocities(new_trajectory)
#         new_time = compute_time(new_trajectory)
#         print('new time:', new_time)
#         if new_time < best_time:
#             best_time = new_time
#             best_trajectory = list(new_trajectory)
#             print("New best time: {}".format(best_time))
#     # Set the anchors
#     for i in range(len(best_trajectory)):
#         if i in anchors:
#             best_trajectory[i].anchor = True
#         else:
#             best_trajectory[i].anchor = False
#     return best_trajectory
    