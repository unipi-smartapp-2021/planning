import rospy
import os
import LTP.RiskFunctions as risk_fun
from LTP.Utils import force_inside_track
from LTP.Trajectory import Trajectory
from LTP.TrackMap import TrackMap
from LTP.Parameters import Parameters
from LTP.SampleTrack import StraightTrackMap
from LTP.ROSInterface import send_trajectory_to_ros_topic, send_risk_to_ros_topic
from planning.msg import LTP_Plan, Risk
from rospy.client import spin
from LTP.PlanStep import PlanStep

class Race:
    def __init__(self, parameters: Parameters):
        self.parameters = parameters
        # Create a Publisher to the LTP_plan topic
        self.trajectory_publisher = rospy.Publisher("ltp_plan", LTP_Plan, queue_size=1)
        self.risk_publisher = rospy.Publisher("risk", Risk, queue_size=1)
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

        # Generate the Trajectory
        self.trajectory = Trajectory(self.parameters)
        #set the risk to the maximum possible
        self.parameters.set_risk(risk_fun.constant(
            1, self.parameters.get_min_risk(), self.parameters.get_max_risk()))
        send_risk_to_ros_topic(self.parameters.get_risk(), self.risk_publisher, Risk)

        #compute the trajectory
        self.trajectory.compute_middle_trajectory(self.track_map)
        #compute the velocities
        self.trajectory.compute_velocities()

        #compute breaking distance
        vel_final = self.trajectory.get_trajectory()[-1].get_velocity()
        breaking_distance = 0.5 * (vel_final**2)/(self.parameters.get_max_deceleration())

        #add last position of trajectory and set to 0
        self.trajectory.trajectory.append(PlanStep((self.trajectory.get_trajectory()[-1].get_position()[0] - breaking_distance, 0), 0, [0,0]))
        #update velocities so that 
        self.trajectory._bound_velocities()

        #send the trajectory
        send_trajectory_to_ros_topic(self.trajectory, self.trajectory_publisher, LTP_Plan)

        #print([planstep.position for planstep in self.trajectory.trajectory])

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
