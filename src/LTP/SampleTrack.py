from LTP.TrackMap import TrackMap
import math
from typing import List, Tuple

def skidpad(first_straight_meters: float, circle_radius: float, second_straight_meters: float, N_cones: Tuple[int, int, int], track_width: int) -> TrackMap:
    """Generates a skidpad track

    Args:
            first_straight_meters (float): length of the first straight track
            circle_radius (float): radius of the curve
            second_straight_meters (float): length of the second straight track

    Returns:
            tuple[list[tuple[float, float]], list[tuple[float, float]]]: [description]
    """
    left_cones: List[Tuple[float, float]] = []
    right_cones: List[Tuple[float, float]] = []
    N_cones_first_straight = N_cones[0]
    N_cones_per_circle = N_cones[1]
    N_cones_second_straight = N_cones[2]

    list_track_map = []

    # First straight track
    for index in range(0, first_straight_meters, first_straight_meters // N_cones_first_straight):
        left_cones.append((0, index))
        right_cones.append((track_width, index))
    list_track_map.append(TrackMap(left_cones, right_cones))

    r_x = right_cones[-1][0]
    r_y = right_cones[-1][1]

    # Righe circle
    circle_track = CircleTrackMap(circle_radius, track_width, N_cones_per_circle, r_x + circle_radius, r_y, start_angle=180, end_angle=-180)
    left_cones = circle_track.left_cones
    right_cones = circle_track.right_cones
    list_track_map.append(TrackMap(left_cones, right_cones))
    list_track_map.append(TrackMap(left_cones, right_cones))

    # Left circle
    circle_track = CircleTrackMap(circle_radius, track_width, N_cones_per_circle, r_x - circle_radius - track_width, r_y, start_angle=0, end_angle=360, swap_cones=True)
    left_cones = circle_track.left_cones
    right_cones = circle_track.right_cones
    list_track_map.append(TrackMap(left_cones, right_cones))
    list_track_map.append(TrackMap(left_cones, right_cones))
    
    r_x = right_cones[-1][0]
    r_y = right_cones[-1][1]
    # Second straight track
    left_cones = []
    right_cones = []
    for index in range(0, second_straight_meters, second_straight_meters // N_cones_second_straight):
        left_cones.append((r_x - track_width, r_y + index))
        right_cones.append((r_x, r_y + index))
    list_track_map.append(TrackMap(left_cones, right_cones))

    return list_track_map


# Generates a straight track of the given length and width.
#   length: the length of the track in meters
#   width: the width of the track in meters
#   N_cones: the number of cones to be placed on the track
#   start_x: the x coordinate of the starting position of the track
#   start_y: the y coordinate of the starting position of the track
def StraightTrackMap(length: float, width: float, num_cones: int, start_x: float, start_y: float) -> TrackMap:
    """Generates a straight track of the given length and width.

    Args:
        length (float): the length of the track in meters
        width (float): the width of the track in meters
        N_cones (int): the number of cones to be placed on the track
        start_x (float): the x coordinate of the starting position of the track (left)
        start_y (float): the y coordinate of the starting position of the track (left)

    Returns:
        tuple[list[tuple[float, float]], list[tuple[float, float]]]: list of left and right cones
    """
    left_cones: list[tuple[float, float]] = []
    right_cones: list[tuple[float, float]] = []
    for index in range(0, length, length // num_cones):
        left_cones.append((start_x, start_y + index))
        right_cones.append((start_x + width, start_y + index))
    return TrackMap(left_cones, right_cones)


def CircleTrackMap(radius: float, width: float, N_cones: int, start_x: float, start_y: float, start_angle: float, end_angle: float, swap_cones=False) -> TrackMap:
    """Generates a curve track of the gicen radius, width and degree

    Args:
            radius (float): radius of the curve
            width (float): width of the track
            degree (int): degree of the curve
            N_cones (int): number of cones in the track
            start_x (float): the x coordinate of the starting position of the track (left)
            start_y (float): the y coordinate of the starting position of the track (left)

    Returns:
            tuple[list[tuple[float, float]], list[tuple[float, float]]]: [description]
    """
    left_cones: list[tuple[float, float]] = []
    right_cones: list[tuple[float, float]] = []
    assert swap_cones == (start_angle < end_angle)
    # TODO: the below 360 should actually be the diff between the end and start angle.. but considering the sign, enjoy :D
    #  ((1 if start_angle < end_angle else -1) * 360)
    for grade in range(start_angle, end_angle, (end_angle - start_angle) // N_cones):
        if not swap_cones:
            # Inner yellow cones
            pos_x = start_x + radius * math.cos(math.radians(grade))
            pos_y = start_y + radius * math.sin(math.radians(grade))
            right_cones.append((pos_x, pos_y))
            # Outer blue cones
            pos_x = start_x + (radius+width) * math.cos(math.radians(grade))
            pos_y = start_y + (radius+width) * math.sin(math.radians(grade))
            left_cones.append((pos_x, pos_y))
        else:
            # Inner yellow cones
            pos_x = start_x + radius * math.cos(math.radians(grade))
            pos_y = start_y + radius * math.sin(math.radians(grade))
            left_cones.append((pos_x, pos_y))
            # Outer blue cones
            pos_x = start_x + (radius+width) * math.cos(math.radians(grade))
            pos_y = start_y + (radius+width) * math.sin(math.radians(grade))
            right_cones.append((pos_x, pos_y))

    return TrackMap(left_cones, right_cones)


if __name__ == '__main__':
    straight_track_map = StraightTrackMap(100, 10, 25, 5, 10)
    straight_track_map.set_car_position(10, 10)
    straight_track_map.show()

    straight_track_map = CircleTrackMap(100, 10, 25, 5, 10)
    straight_track_map.set_car_position(10, 10)
    straight_track_map.show()
