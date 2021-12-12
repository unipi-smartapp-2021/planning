
"""
    Module MidTrajectory will compute the trajectory of the car
"""
from typing import List, Tuple
from LTP.TrackMap import TrackMap
from LTP.PlanStep import PlanStep
from LTP.Utils import euclidean_distance, find_closest_point, compute_middle_point, compute_spline, find_line
from math import atan2, sin, cos, sqrt, inf
import copy

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

    def force_inside_track(self, track_map):
        # It forces all the points of the trajectory to be inside the track,
        # Also it make sure that the lines connecting the points of the trajectory is inside the track map

        # 1. Make sure that all the points are inside the track
        for i in range(0, len(self.trajectory)):
            self.trajectory[i].position = track_map.force_point_inside_track(self.trajectory[i].position)

        new_trajectory = []

        # 2. Make sure that the lines connecting the points are inside the track
        # TODO: we must not consider the last point in case of a non-closed loop:
        # e.g.: it should be len(self.trajectory) - 1 if not closed (Take this from self.parameters?)
        for i in range(0, len(self.trajectory)):
            # We consider two points at a time, the idea is that if the line goes outside we add middle points
            # between the two points until the line is inside the track
            new_trajectory = new_trajectory + self._force_line_inside_track(self.trajectory[i].position, self.trajectory[(i + 1) % len(self.trajectory)].position, track_map)
        return new_trajectory

    def _force_line_inside_track(self, curr_pos: Tuple[float, float], next_pos: Tuple[float, float], track_map) -> List[PlanStep]:
        """
            Force the line between two points to be inside the track
            :param curr_pos: current position
            :param next_pos: next position
            :param track_map: track map
            :return: a list of points that are inside the track
        """
        new_trajectory = []
        if track_map.is_line_inside_track(curr_pos, next_pos):
            new_trajectory = new_trajectory + [PlanStep(curr_pos)]
        else:
            # We need to add a middle point
            middle_point = compute_middle_point(curr_pos, next_pos)
            m_traj, q_traj = find_line(curr_pos, next_pos)
            # We want to a line passing through middle_point and orthogonal to (m_traj, q_traj)
            m_orth = -1/m_traj
            q_orth = middle_point[0]/m_traj + middle_point[1]
            middle_point = track_map.force_point_inside_track(middle_point, project_line = (m_orth, q_orth))
            print("new middle point: ", middle_point)
            new_trajectory = new_trajectory + self._force_line_inside_track(curr_pos, middle_point, track_map)
            new_trajectory = new_trajectory + self._force_line_inside_track(middle_point, next_pos, track_map)
        return new_trajectory

    def compute_velocities(self):
        """
        Compute the velocities of the trajectory.
        :param trajectory: the trajectory to compute the velocities
        :return: the trajectory with the velocities
        """
        if (len(self.trajectory) < 3):
            for i in range(0, len(self.trajectory)):
                self.trajectory[i].velocity = self.parameters.get_min_velocity()
        else:
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
        if len(self.trajectory) >= 2:
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
                    new_velocity = max(self._reduce_final_velocity(current_pos, next_pos, current_velocity, self.parameters.get_max_acceleration()), self.parameters.get_min_velocity())
                    print(f"Reducing final velocity from {new_trajectory[(i+1) % len(self.trajectory)].velocity} to {new_velocity}")
                    new_trajectory[(i+1) % len(self.trajectory)].velocity = new_velocity
                elif -self.parameters.get_max_deceleration() < required_acc:
                    # Given the max_deceleration of our car we cannot slow down to the final velocity
                    # We then decrease the initial velocity to the highest velocity we can reach starting from the final point
                    # and assuming that our acceleration is the negative of the maximum deceleration
                    new_velocity = max(self._reduce_final_velocity(next_pos, current_pos, next_velocity, -self.parameters.get_max_deceleration()), self.parameters.get_min_velocity())
                    print(f"Reducing initial velocity from {new_trajectory[i % len(self.trajectory)].velocity} to {new_velocity}")
                    new_trajectory[i % len(self.trajectory)].velocity = new_velocity
            return new_trajectory
        else:
            return copy.deepcopy(self.trajectory)

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
        return 1/K if K != 0 else inf

    def _compute_angles(self):
        for i in range(0, len(self.trajectory)):
            current_pos = self.trajectory[i].position
            next_pos = self.trajectory[(i+1) % len(self.trajectory)].position
            current_velocity = self.trajectory[i].velocity
            angle = atan2(next_pos[1] - current_pos[1], next_pos[0] - current_pos[0])
            v_x = current_velocity * cos(angle)
            v_y = current_velocity * sin(angle)
            self.trajectory[i].velocity_vector = (v_x, v_y)