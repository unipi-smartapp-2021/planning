import rospy
import os
import LTP.RiskFunctions as risk_fun
from LTP.Utils import force_inside_track
from LTP.Trajectory import Trajectory
from LTP.TrackMap import TrackMap
from LTP.Parameters import Parameters
from LTP.SampleTrack import StraightTrackMap
from LTP.ROSInterface import send_trajectory_to_ros_topic
from planning.msg import LTP_Plan
from rospy.client import spin


class Race:
    def __init__(self, parameters: Parameters):
        self.parameters = parameters
        # Create a Publisher to the LTP_plan topic
        self.publisher = rospy.Publisher("ltp_plan", LTP_Plan, queue_size=1)
        #Initialize ROS node
        rospy.init_node('ltp', anonymous=False)

    def race_loop():
        raise NotImplemented()


class Acceleration(Race):
    def __init__(self, parameters):
        super().__init__(parameters)

    def race_loop(self):
        # TODO: In parameters we assume to have the length and width of the Acceleration race
        # TODO think when the track finishes
        race_length = self.parameters.get_track_length()
        race_width = self.parameters.get_track_width()
        intra_cone_distance = self.parameters.get_intra_cone_distance()

        # Generate the track map given the info
        self.track_map = TrackMap()
        print(os.getcwd())
        self.track_map.load_track("./src/LTP/tests/tracks/acceleration.json")
        print(self.track_map.get_right_cones())
        # Generate the Trajectory
        self.trajectory = Trajectory(self.parameters)
        #set the risk to the maximum possible
        self.parameters.set_risk(risk_fun.constant(
            1, self.parameters.get_min_risk(), self.parameters.get_max_risk()))
        print(self.parameters.max_velocity_risk)
        #compute the trajectory
        self.trajectory.compute_middle_trajectory(self.track_map)
        #compute the velocities
        self.trajectory.compute_velocities()
        #send the trajectory
        send_trajectory_to_ros_topic(self.trajectory, self.publisher, LTP_Plan)

class SkidPad(Race):
    def __init__(self, parameters):
        super().__init__(parameters)

    def race_loop(self):
        pass


class AutoCross(Race):
    def __init__(self, parameters):
        super().__init__(parameters)

    def race_loop(self):
        pass


class TrackDrive(Race):
    def __init__(self, parameters):
        super().__init__(parameters)

    def race_loop(self):
        pass
