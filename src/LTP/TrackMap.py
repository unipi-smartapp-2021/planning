"""
    Module TrackMap manages Trackmap object and their representation
"""

from math import inf
from typing import List, Tuple
import matplotlib.pyplot as plt
import json

from LTP.Utils import compute_distance

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

    def force_inside_track(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """
            Force a point to be inside the track
        """
        nearest_left_cone = self.get_nearest_left_cone(point)
        nearest_right_cone = self.get_nearest_right_cone(point)
        # Check if the point is already inside the cones
        min_x = min(nearest_left_cone[0], nearest_right_cone[0])
        max_x = max(nearest_left_cone[0], nearest_right_cone[0])
        min_y = min(nearest_left_cone[1], nearest_right_cone[1])
        max_y = max(nearest_left_cone[1], nearest_right_cone[1])
        # If the point is outside the cones, we force it inside
        if point[0] < min_x:
            point = (min_x, point[1])
        if point[0] > max_x:
            point = (max_x, point[1])
        if point[1] < min_y:
            point = (point[0], min_y)
        if point[1] > max_y:
            point = (point[0], max_y)
        return point

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
