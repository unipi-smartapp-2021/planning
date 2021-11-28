"""
    Module TrackMap manages Trackmap object and their representation
"""

from math import inf
from typing import List, Tuple
import matplotlib.pyplot as plt
import json
import numpy as np

from LTP.Utils import compute_distance, find_line

def remove_duplicates(lst: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
        remove duplicates cone from a list of cones
    """
    new_list = []
    def is_equal(x1, x2):
        return x1[0] == x2[0] and x1[1] == x2[1]
    for i in range(len(lst)):
        count = 0
        for j in range(i+1, len(lst)):
            if is_equal(lst[i], lst[j]):
                count += 1
        if count == 0:
            new_list.append(lst[i])
    return new_list

class TrackMap:
    """
        Represent a Track Map defined by the left and right cones positions
        Each cone is represented by a tuple (x, y) which represent its position
        inside a Cartesian Plane.
    """
    def __init__(self, left_cones: List[Tuple[float, float]] = [], right_cones: List[Tuple[float, float]] = []):
        self.left_cones = remove_duplicates(left_cones)
        self.right_cones = remove_duplicates(right_cones)
        self.car_position = None
    
    def is_line_inside_track(self, curr_pos, next_pos):
        # Find the line between the two points
        m, q = find_line(curr_pos, next_pos)
        # Find all the cones in between the curr and next pos
        x_start = min(curr_pos[0], next_pos[0])
        x_end = max(curr_pos[0], next_pos[0])
        N = 50 # TODO: Find a better way to do this, e.g: recursively
        # increase the number of points until an epsilon is reached
        for x in np.linspace(x_start, x_end, N):
            y = m * x + q
            if not self.is_point_inside_track((x, y)):
                return False
        return True

    def is_point_in_path(self, x: int, y: int, poly) -> bool:
        # Ref: https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
        # Determine if the point is in the polygon.
        #
        # Args:
        #   x -- The x coordinates of point.
        #   y -- The y coordinates of point.
        #   poly -- a list of tuples [(x, y), (x, y), ...]
        #
        # Returns:
        #   True if the point is in the path or is a corner or on the boundary
        num = len(poly)
        j = num - 1
        c = False
        for i in range(num):
            if (x == poly[i][0]) and (y == poly[i][1]):
                # point is a corner
                return True
            if ((poly[i][1] > y) != (poly[j][1] > y)):
                slope = (x-poly[i][0])*(poly[j][1]-poly[i][1])-(poly[j][0]-poly[i][0])*(y-poly[i][1])
                if slope == 0:
                    # point is on boundary
                    return True
                if (slope < 0) != (poly[j][1] < poly[i][1]):
                    c = not c
            j = i
        return c

    def is_point_inside_track(self, point: Tuple[float, float]) -> bool:
        # We check for every quadrilater if the point is inside it
        for curr_left, curr_right, next_left, next_right in zip(self.left_cones, self.right_cones, self.left_cones[1:], self.right_cones[1:]):
            poly = [curr_left, curr_right, next_right, next_left]
            if self.is_point_in_path(point[0], point[1], poly):
                return True
        return False

    def find_lines_intersection(self, line1: Tuple[float, float], line2: Tuple[float, float]) -> Tuple[float, float]:
        # Find the intersection of two lines
        #
        # Args:
        #   line1 -- a tuple (m, q) of the first line
        #   line2 -- a tuple (m, q) of the second line
        #
        # Returns:
        #   The intersection of the two lines
        # TODO: Add check of None m values..
        a = line1[0]
        c = line1[1]
        b = line2[0]
        d = line2[1]
        x = (d-c)/(a-b)
        y = a*x + c
        return x, y

    # TODO: Also conside the track width and risk to better position the point
    # right now we are just placing it on the border..
    def force_point_inside_track(self, point: Tuple[float, float], project_line: Tuple[float, float] = None) -> Tuple[float, float]:
        """
            Force a point to be inside the track
        """
        if not self.is_point_inside_track(point):
            # In case we don't have a direction we just use the nearest left cone
            if project_line is None:
                nearest_left_cone, _ = self.get_nearest_left_cone(point)
                nearest_right_cone, _ = self.get_nearest_right_cone(point)
                point = (nearest_left_cone[0] + nearest_right_cone[0]) / 2, (nearest_left_cone[1] + nearest_right_cone[1]) / 2
                return point
            # If we have a direction we project the point on the track
            best_middle_point = None
            best_middle_point_distance = inf
            # TODO: do it with an index to also consider the case of closed loops
            for curr_left, curr_right, next_left, next_right in zip(self.left_cones, self.right_cones, self.left_cones[1:], self.right_cones[1:]):
                m_left, q_left = find_line(curr_left, next_left)
                m_right, q_right = find_line(curr_right, next_right)
                intersection_left = self.find_lines_intersection((m_left, q_left), project_line)
                intersection_right = self.find_lines_intersection((m_right, q_right), project_line)
                if intersection_left[0] is not None and intersection_right[0] is not None:
                    middle_point = (intersection_left[0] + intersection_right[0]) / 2, (intersection_left[1] + intersection_right[1]) / 2
                    middle_point_distance = compute_distance(point, middle_point)
                    if middle_point_distance < best_middle_point_distance:
                        best_middle_point = middle_point
                        best_middle_point_distance = middle_point_distance
            return best_middle_point

    

    def _compute_nearest_cone(self, position: Tuple[float, float], cones: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
            Compute the nearest cone to the given position
        """
        nearest_cone = None
        min_distance = inf
        for cone in cones:
            distance = compute_distance(cone, position)
            if distance < min_distance:
                min_distance = distance
                nearest_cone = cone
        return nearest_cone

    def get_nearest_left_cone(self, position: Tuple[float, float]) -> Tuple[float, float]:
        """
            Get the nearest left cone to the given position
        """
        return self._compute_nearest_cone(position, self.left_cones)

    def get_nearest_right_cone(self, position: Tuple[float, float]) -> Tuple[float, float]:
        """
            Get the nearest right cone to the given position
        """
        return self._compute_nearest_cone(position, self.right_cones)

    def get_track_width(self):
        """
            Compute the track width as the distance between the first 2 cones (TODO: There could be missing cones)
        """
        return compute_distance(self.left_cones[0], self.right_cones[0])

    def load_track(self, file_path):
        """
            Load a json file containing the left and right cones to initialize class' values
        """
        with open(file_path) as f:
            data = json.load(f)
        for yellow_cone in data['yellow_cones']:
            self.left_cones.append((yellow_cone['x'], yellow_cone['y']))
        for blue_cone in data['blue_cones']:
            self.right_cones.append((blue_cone['x'], blue_cone['y']))
        self.left_cones = remove_duplicates(self.left_cones)
        self.right_cones = remove_duplicates(self.right_cones)
    
    def get_left_cones(self):
        """
            Obtain all Left cones inside the TrackMap
        """
        return self.left_cones

    def get_right_cones(self):
        """
            Obtain all Right Cones inside the TrackMap
        """
        return self.right_cones
    
    def set_car_position(self, coordinate):
        """set the car position

        Args:
            coordinate ([type]): [description]
        """
        self.car_position = coordinate
    
    def get_car_position(self):
        """
            return car position
        """

    def show(self):
        """
            Show Graphically the Track Map using Matplotlib
        """
        plt.figure()
        plt.title('Track Map')
        plt.xlabel('Position x')
        plt.ylabel('Position y')
        plt.scatter([pos_x for pos_x, _ in self.left_cones], [pos_y for _, pos_y in self.left_cones],
                    color='blue', label='Left Cones')
        plt.scatter([pos_x for pos_x, _ in self.right_cones], [pos_y for _, pos_y in self.right_cones],
                    color='yellow', label='Right Cones')
        if self.car_position is not None:
            plt.scatter(self.car_position[0], self.car_position[1], color='red', label='Car Pos')
        plt.show()
