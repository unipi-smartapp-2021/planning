
"""
    Module MidTrajectory will compute the trajectory of the car
"""
from typing import List
from LTP.TrackMap import TrackMap
from LTP.PlanStep import PlanStep
from LTP.Utils import euclidean_distance, find_closest_point, compute_middle_point, compute_spline
from math import atan2, sin, cos, sqrt


class Trajectory:
    """
        Represent an abstract Trajectory class used to represent
        the Trajectory desired for the car
    """

    def __init__(self, parameters, distance_fun=euclidean_distance):
        self.distance_fun = distance_fun
        self.trajectory: List[PlanStep] = []
        self.parameters = parameters

    def get_trajectory(self):
        """
            Get the computed trajectory for the car
        """
        return self.trajectory

    def set_trajectory(self, new_trajectory: List[PlanStep]):
        """set new trajectory

        Args:
            List (PlanStep): new trajectory
        """
        self.trajectory = new_trajectory

    def compute_middle_trajectory(self, track_map: TrackMap):
        cones_left = track_map.get_left_cones()
        cones_right = track_map.get_right_cones()

        # reinitialize trajectory
        self.trajectory = []

        for left_point in cones_left:
            right_point = cones_right[find_closest_point(left_point, cones_right, self.distance_fun)]
            self.trajectory.append(PlanStep(compute_middle_point(left_point, right_point)))

    def compute_velocities(self):
        """
        Compute the velocities of the trajectory.
        :param trajectory: the trajectory to compute the velocities
        :return: the trajectory with the velocities
        """
        # We need the first and second derivative of the trajectory to compute the Curvature needed
        # for the maximum velocity
        _, f_x, f_y, df_x, df_y, ddf_x, ddf_y = compute_spline(self.trajectory)
        for i in range(0, len(self.trajectory)):
            curvature = self._compute_radius(f_x[i], f_y[i], df_x[i], df_y[i], ddf_x[i], ddf_y[i])
            self.trajectory[i].velocity = self._compute_velocity(curvature)
        # Assert that the velocities are not too high
        self._bound_velocities()
        # Compute the velocity vector
        self._compute_angles()

    def compute_time(self):
        time = 0
        for index, step in enumerate(self.trajectory):
            current_pos = step.position
            next_pos = self.trajectory[(index+1) % len(self.trajectory)].position
            current_velocity = step.velocity
            next_velocity = self.trajectory[(index+1) % len(self.trajectory)].velocity
            avg_velocity = (current_velocity + next_velocity) / 2
            time += euclidean_distance(current_pos, next_pos) / avg_velocity
        return time

    # We need to reduce the final velocity
    # 0.5 * a * t^2 + v_1 * t + (curr_pos - next_pos) = 0
    # Solve for t then v_f = v_1 + a*t
    def _reduce_final_velocity(self, current_pos, next_pos, current_velocity, max_acceleration):
        new_time = (-2*current_velocity + sqrt((2*current_velocity)**2 + 8*max_acceleration * euclidean_distance(current_pos, next_pos))) / (2*max_acceleration)
        new_velocity = current_velocity + max_acceleration * new_time
        return new_velocity

    def _bound_velocities(self):
        new_trajectory = list(self.trajectory)
        # TODO: note the 2*len(trajectory) makes sense only in a closed loop (is this assumption correct?)
        for i in range(0, 2*len(self.trajectory)):
            # We check if from the current velocity and position we can reach the next velocity given the car constraints
            current_pos = new_trajectory[i % len(new_trajectory)].position
            next_pos = new_trajectory[(i+1) % len(new_trajectory)].position
            current_velocity = new_trajectory[i % len(new_trajectory)].velocity
            next_velocity = new_trajectory[(i+1) % len(new_trajectory)].velocity
            avg_velocity = (current_velocity + next_velocity) / 2
            time = euclidean_distance(current_pos, next_pos) / avg_velocity
            required_acc = (next_velocity - current_velocity) / time # v_f = v_i + a*t => a = v_f - v_i / t
            if required_acc > self.parameters.get_max_acceleration():
                # Given the max_acceleration of our car we cannot reach the final velocity
                # We then decrease the final velocity to the highest velocity we can reach
                new_velocity = min(self._reduce_final_velocity(current_pos, next_pos, current_velocity, self.parameters.get_max_acceleration()), self.parameters.get_min_velocity())
                print(f"Reducing final velocity from {new_trajectory[(i+1) % len(self.trajectory)].velocity} to {new_velocity}")
                new_trajectory[(i+1) % len(self.trajectory)].velocity = new_velocity
            elif -self.parameters.get_max_deceleration() < required_acc:
                # Given the max_deceleration of our car we cannot slow down to the final velocity
                # We then decrease the final velocity to the highest velocity we can reach starting from the final point
                # and assuming that our acceleration is the negative of the maximum deceleration
                new_velocity = min(self._reduce_final_velocity(next_pos, current_pos, next_velocity, -self.parameters.get_max_deceleration()), self.parameters.get_min_velocity())
                print(f"Reducing initial velocity from {new_trajectory[i % len(self.trajectory)].velocity} to {new_velocity}")
                new_trajectory[i % len(self.trajectory)].velocity = new_velocity
        return new_trajectory

    def _compute_velocity(self, curvature: float) -> float:
        """
            Compute Velocity of Curvature given Grip force, mass m and curvature R
            v_max = sqrt((R*grip_force)/mass)

            Param:
                -curvature(float): radius of Curvature
                -grip_force(float): grip force provided by the car
                -mass(float): mass of the Car
        """
        grip_force = self.parameters.get_grip_force()
        mass = self.parameters.get_mass()
        #get maximum velocity according to the risk
        max_velocity = self.parameters.get_max_velocity_risk()
        return min(sqrt((grip_force * curvature) / mass), max_velocity)

    def _compute_radius(self, f_x: float, f_y: float, df_x: float, df_y: float, ddf_x: float, ddf_y: float) -> float:
        """
        Compute the radius of the trajectory
        :param f_x: f(x)
        :param f_y: f(y)
        :param df_x: first derivative of x
        :param df_y: first derivative of y
        :param ddf_x: second derivative of x
        :param ddf_y: second derivative of y
        :return: the radius of the trajectory
        """
        num = abs(df_x * ddf_y - df_y * ddf_x)
        den = (df_x**2 + df_y**2)**(3/2)
        K = num / den
        return 1/K

    def _compute_angles(self):
        for i in range(0, len(self.trajectory)):
            current_pos = self.trajectory[i].position
            next_pos = self.trajectory[(i+1) % len(self.trajectory)].position
            current_velocity = self.trajectory[i].velocity
            angle = atan2(next_pos[1] - current_pos[1], next_pos[0] - current_pos[0])
            v_x = current_velocity * cos(angle)
            v_y = current_velocity * sin(angle)
            self.trajectory[i].velocity_vector = (v_x, v_y)