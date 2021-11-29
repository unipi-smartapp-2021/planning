from typing import List, Tuple
from LTP.PlanStep import PlanStep
from scipy import interpolate
import matplotlib.pyplot as plt
import json
import numpy as np

"""
    Utility module used to define utility functions
"""

# TODO: Assert that also the line connecting the two points is inside
def force_inside_track(track_map, trajectory: List[PlanStep]) -> List[PlanStep]:
    """
    Force the trajectory to be inside the track

    Args:
        track_map (TrackMap): track map
        trajectory (List[PlanStep]): trajectory

    Returns:
        List[PlanStep]: trajectory
    """
    new_trajectory = []
    for plan_step in trajectory:
        new_position = track_map.force_inside_track(plan_step.position)
        new_trajectory.append(PlanStep(new_position, plan_step.velocity, list(plan_step.velocity_vector)))
    return new_trajectory

def convert_to_kmh(meters_second):
    """
        Convert m/s to km/h returning a floating value representing
        velocity in kilometers to hour

        Param:
            -meters_second(floating): meters second velocity
    """
    return meters_second * 3.6

def euclidean_distance(point_1, point_2):
    """
        Compute the euclidean distance between two points
    """
    return ((point_1[0] - point_2[0])**2 + (point_1[1] - point_2[1])**2)**0.5

def euclidean_distance_no_sqrt(point_1, point_2):
    """
        Compute the "euclidean" distance (without sqrt) between two points
    """
    return (point_1[0] - point_2[0])**2 + (point_1[1] - point_2[1])**2

def compute_distance(point_1: Tuple[float,float], point_2: Tuple[float, float], distance_function=euclidean_distance) -> float:
    """compute the distance between two points

    Args:
        x1 (tuple[float,float]): [description]
        x2 (tuple[float, float]): [description]
        distance_function ([type], optional): [description]. Defaults to euclidean_distance.

    Returns:
        float: [description]
    """
    return distance_function(point_1, point_2)

def find_closest_point(point, points, distance_function=euclidean_distance_no_sqrt) -> int:
    """find the closest point to point in points

    Args:
        point ([type]): [description]
        points ([type]): [description]
        distance_function ([type], optional): [description]. Defaults to euclidean_distance_no_sqrt.

    Returns:
        int: [description]
    """
    min_distance = distance_function(point, points[0])
    min_index = 0
    for index, candidate_point in enumerate(points):
        distance = distance_function(point, candidate_point)
        if distance < min_distance:
            min_distance = distance
            min_index = index
    return min_index

def compute_middle_point(left_point, right_point):
    """
        Find the middle point given two points
    """
    return (left_point[0] + right_point[0]) / 2, (left_point[1] + right_point[1]) / 2

def compute_spline(points: List[PlanStep]):
    """compute spline function given a list of PlanStep

    Args:
        points (List[PlanStep]): set of PlanStep

    Returns:
        [type]: [description]
    """
    xs = [plan_step.position[0] for plan_step in points]
    ys = [plan_step.position[1] for plan_step in points]

    tck,u=interpolate.splprep([xs,ys],s=0.0)

    x_i,y_i= interpolate.splev(u,tck)
    dx_i,dy_i= interpolate.splev(u,tck, der=1)
    ddx_i,ddy_i= interpolate.splev(u,tck, der=2)

    return tck, x_i,y_i,dx_i,dy_i,ddx_i,ddy_i

def find_lines_intersection(line1: Tuple[float, float], line2: Tuple[float, float]) -> Tuple[float, float]:
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

def find_line(p1, p2):
    """Find line between two points

    Args:
        p1 ([type]): [description]
        p2 ([type]): [description]

    Returns:
        [type]: [description]
    """
    if p1[0] == p2[0]:
        return (0, p1[0])
    else:
        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = p1[1] - m * p1[0]
        return (m, b)


def serialize_to_file(left_cones: List[Tuple[float, float]], right_cones: List[Tuple[float, float]], trajectory: List[PlanStep], file_name: str):
    d = {
        'left_cones': {
            'x': list(map(lambda x: x[0], left_cones)),
            'y': list(map(lambda x: x[1], left_cones))
        },
        'right_cones': {
            'x': list(map(lambda x: x[0], right_cones)),
            'y': list(map(lambda x: x[1], right_cones))
        },
        'trajectory': {
            'positions': {
                'x': list(map(lambda x: x.position[0], trajectory)),
                'y': list(map(lambda x: x.position[1], trajectory))
            },
            'velocities': list(map(lambda x: x.velocity, trajectory)),
        }
    }

    with open(file_name + ".JSON", 'w') as f:
        json.dump(d, f)