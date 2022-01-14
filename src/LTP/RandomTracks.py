from TrackMap import TrackMap
import random
import numpy as np

def generate_random_track() -> TrackMap:
    # The idea is to use a random polynomial to generate a random track.
    k = random.randint(1, 5)
    poly_terms = [10*(random.random()*2-1) for _ in range(k+1)]
    poly_f = lambda x: sum(p * (x ** i) for i, p in enumerate(poly_terms))
    # Now we sample the track at random points.
    N_cones = random.randint(5, 8)
    cone_x = sorted(np.random.uniform(-10, 10, N_cones))
    cone_y = [poly_f(x) for x in cone_x]

    # Normalize the y value to be between 0 and 30
    y_min = min(cone_y)
    y_max = max(cone_y)
    y_range = y_max - y_min
    y_norm = lambda y: (y - y_min) / y_range * 30
    cone_y = [y_norm(y) for y in cone_y]

    # Apply a random horizontal flip
    if random.random() < 0.5:
        cone_x = [-x for x in cone_x]
        cone_y = [y for y in cone_y]
    # Apply a random vertical flip
    if random.random() < 0.5:
        cone_x = [x for x in cone_x]
        cone_y = [-y for y in cone_y]

    # Now we construct the list of left cones (x, y) and right cones (x, y).
    left_cones = [(x, y) for x, y in zip(cone_x, cone_y)]
    right_cones = [(x, y-5) for x, y in zip(cone_x, cone_y)]

    # Now we construct the track.
    track = TrackMap(left_cones, right_cones, (right_cones[0][0], right_cones[0][1]+2.5), (-1, 2.5))
    return track