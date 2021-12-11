from LTP.TrackMap import TrackMap
import rospy
from geometry_msgs.msg import PoseArray, Pose, Point

# TODO: Right now there is no way to know when the Race is finished
#       and also to know when we complete a lap.
class RaceState():
    def __init__(self, subscribe=False):
        self.track_map = TrackMap()
        self.track_map_updated = False
        self.finished = False
        self.current_lap = 1

        # Subscribe to the left and right cones
        self.is_left_cones_updated = False
        self.is_right_cones_updated = False
        self.left_cones = []
        self.right_cones = []
        rospy.Subscriber("/cone_right", PoseArray, self.update_right_cones)
        rospy.Subscriber("/cone_left", PoseArray, self.update_left_cones)

    def is_last_lap(self):
        return True # TODO: Right now there is no way to know the current lap (KB)

    def update_left_cones(self, msg):
        poses = msg.poses
        left_cones = []
        for pose in poses:
            point: Point = pose.position
            left_cones.append((point.x, point.y))
        self.is_left_cones_updated = True
        self.try_update_track_map()

    def update_right_cones(self, msg):
        poses = msg.poses
        right_cones = []
        for pose in poses:
            point: Point = pose.position
            right_cones.append((point.x, point.y))
        self.is_right_cones_updated = True
        self.try_update_track_map()

    def try_update_track_map(self):
        if self.is_left_cones_updated and self.is_right_cones_updated:
            self.track_map = TrackMap(self.left_cones, self.right_cones)
            self.track_map_updated = True
            self.is_left_cones_updated = False
            self.is_right_cones_updated = False

    def get_finished_status(self):
        return self.finished

    def get_track_map(self):
        return self.track_map

    def is_track_map_new(self):
        return self.track_map_updated