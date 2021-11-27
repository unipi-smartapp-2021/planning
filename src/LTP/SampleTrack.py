from LTP.TrackMap import TrackMap
import math

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


def CircleTrackMap(radius: float, width: float, N_cones: int, start_x: float, start_y: float) -> TrackMap:
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

    for grade in range(0, 360, 360 // N_cones):
        # Inner yellow cones
        pos_x = start_x + radius * math.cos(math.radians(grade))
        pos_y = start_y + radius * math.sin(math.radians(grade))
        right_cones.append((pos_x, pos_y))
        # Outer blue cones
        pos_x = start_x + (radius+width) * math.cos(math.radians(grade))
        pos_y = start_y + (radius+width) * math.sin(math.radians(grade))
        left_cones.append((pos_x, pos_y))

    return TrackMap(left_cones, right_cones)


if __name__ == '__main__':
    straight_track_map = StraightTrackMap(100, 10, 25, 5, 10)
    straight_track_map.set_car_position(10, 10)
    straight_track_map.show()

    straight_track_map = CircleTrackMap(100, 10, 25, 5, 10)
    straight_track_map.set_car_position(10, 10)
    straight_track_map.show()
