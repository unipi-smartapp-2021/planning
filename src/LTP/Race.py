import rospy
import os
import LTP.RiskFunctions as risk_fun
from LTP.Utils import force_inside_track, serialize_to_file, reorder_cones
from LTP.Trajectory import Trajectory
from LTP.TrackMap import TrackMap, load_track
from LTP.Parameters import Parameters
from LTP.SampleTrack import StraightTrackMap, skidpad
from LTP.ROSInterface import send_trajectory_to_ros_topic, send_risk_to_ros_topic
from planning.msg import LTP_Plan, Risk
from rospy.client import spin
from LTP.PlanStep import PlanStep
from LTP.RaceState import RaceState


class Race:
    def __init__(self, parameters: Parameters, race_state: RaceState):
        self.parameters = parameters
        self.race_state = race_state
        # Create a Publisher to the LTP_plan topic
        self.trajectory_publisher = rospy.Publisher(
            "ltp_plan", LTP_Plan, queue_size=1)
        self.risk_publisher = rospy.Publisher("ltp_risk", Risk, queue_size=1)
        # Initialize ROS node
        rospy.init_node('ltp', anonymous=False)
        self.rate = rospy.Rate(5)

    def race_loop():
        raise NotImplemented()


class Acceleration(Race):
    def __init__(self, parameters: Parameters, race_state: RaceState):
        super().__init__(parameters, race_state)

    def race_loop(self):
        rospy.loginfo("wainting for cones")
        while self.race_state.get_finished_status() == False:
            #check if race is finished
            if self.race_state.get_car_position()[0] > self.parameters.get_acc_track_acc_length():
                self.race_state.set_finished_status(True)
                
            if self.race_state.is_track_map_new():
                
                track_map = self.race_state.get_track_map()
                trajectory = Trajectory(self.parameters)
                
                # update risk
                # set the risk to the minimum possible
                self.parameters.set_risk(risk_fun.constant(
                    0, self.parameters.get_min_risk(), self.parameters.get_max_risk()))
                send_risk_to_ros_topic(self.parameters.get_risk(),
                                    self.risk_publisher, Risk)

                rospy.loginfo(
                    f"received track map with {len(track_map.get_left_cones())} left cones and {len(track_map.get_left_cones())} right cones")

                if len(track_map.get_left_cones()) > 0 and len(track_map.get_right_cones()) > 0:
                    # Compute the trajectory
                    trajectory.compute_middle_trajectory(track_map) 
                    # compute the velocities
                    trajectory.compute_velocities()

                    # Remove point that are behind us
                    car_x = self.race_state.get_car_position()[0]
                    trajectory.trajectory = list(filter(lambda x: x.position[0] >= car_x, trajectory.trajectory))
                    if len(trajectory.trajectory) > 0:
                        # # send the trajectory
                        # print("car position:", self.race_state.get_car_position())
                        # print("final step x: ", trajectory.trajectory[-1].position[0])
                        send_trajectory_to_ros_topic(
                            trajectory, self.trajectory_publisher, LTP_Plan)

                        rospy.loginfo(
                            f"Sent plan with {len(trajectory.get_trajectory())} steps")
            self.rate.sleep()
            
        car_x = self.race_state.get_car_position()[0]
        car_y = self.race_state.get_car_position()[1]
        
        # compute breaking distance
        trajectory = Trajectory(self.parameters)
        #vel_final = trajectory.get_trajectory()[-1].get_velocity()
        #breaking_distance = 0.5 * (vel_final**2) / \
        #    (self.parameters.get_max_deceleration())

        breaking_distance = -20
        
        # add last position of trajectory and set to 0 (2 for avoiding bug)
        trajectory.trajectory.append(PlanStep((car_x - breaking_distance, car_y), 2, [2, 0]))
        # update velocities so that we sure we stop
        trajectory._bound_velocities()

        # add 0 velocity points
        STEP = 5
        METERS_END = self.parameters.get_acc_track_dec_length() + breaking_distance
        

        for i in range(1, int(METERS_END//STEP)):
            trajectory.trajectory.append(PlanStep(
                (trajectory.get_trajectory()[-1].get_position()[0] + STEP, car_y), 2, [2, 0]))
        
        # send the trajectory
        send_trajectory_to_ros_topic(
            trajectory, self.trajectory_publisher, LTP_Plan)

        rospy.loginfo(f"Final plan with {len(trajectory.get_trajectory())} steps")
                        
        rospy.loginfo("race finished")

    """
    def race_loop(self):
        # TODO: In parameters we assume to have the length and width of the Acceleration race
        # TODO think when the track finishes

        # Generate the track map given the info
        track_map = load_track("./src/LTP/tests/tracks/acc_slam.json")

        # Generate the Trajectory
        trajectory = Trajectory(self.parameters)
        # set the risk to the maximum possible
        self.parameters.set_risk(risk_fun.constant(
            1, self.parameters.get_min_risk(), self.parameters.get_max_risk()))
        send_risk_to_ros_topic(self.parameters.get_risk(),
                               self.risk_publisher, Risk)

        # compute the trajectory
        trajectory.compute_middle_trajectory(track_map)
        # compute the velocities
        trajectory.compute_velocities()

        # compute breaking distance
        vel_final = trajectory.get_trajectory()[-1].get_velocity()
        breaking_distance = 0.5 * (vel_final**2) / \
            (self.parameters.get_max_deceleration())

        # add last position of trajectory and set to 0
        trajectory.trajectory.append(PlanStep((trajectory.get_trajectory(
        )[-1].get_position()[0] - breaking_distance, 0), 0, [0, 0]))
        # update velocities so that we sure we stop
        trajectory._bound_velocities()

        # add 0 velocity points for 50 meters every STEP meters
        STEP = 5
        METERS_END = self.parameters.get_acc_track_dec_length() + breaking_distance

        for i in range(1, int(METERS_END//STEP)):
            trajectory.trajectory.append(PlanStep(
                (trajectory.get_trajectory()[-1].get_position()[0] + STEP, 0), 0, [0, 0]))

        # send the trajectory
        send_trajectory_to_ros_topic(
            trajectory, self.trajectory_publisher, LTP_Plan)

        serialize_to_file(track_map.get_left_cones(
        ), track_map.get_right_cones(), trajectory.get_trajectory(), "casino")
        #print([planstep.position for planstep in trajectory.trajectory])
    """


class SkidPad(Race):
    def __init__(self, parameters: Parameters, race_state: RaceState):
        super().__init__(parameters, race_state)

    def race_loop(self):
        risk = risk_fun.compute_risk_skidpad()
        send_risk_to_ros_topic(risk, self.risk_publisher, Risk)
        # TODO: In theory the ParameterServer from the KB should update the risk by subscribing to the risk topic
        self.parameters.set_risk(risk)

        # TODO: Read these parameters from self.parameters (KB)
        track_maps = skidpad(first_straight_meters=10, circle_radius=30,
                             second_straight_meters=10, N_cones=(5, 25, 5), track_width=3)

        def compute_trajectory(track_map):
            # Compute trajectory for first straight
            first_straight_track_map = track_map
            trajectory = Trajectory(self.parameters)
            # Compute the trajectory
            trajectory.compute_middle_trajectory(track_map)
            # compute the velocities
            trajectory.compute_velocities()
            return trajectory

        # Merge together all the trajectories
        trajectory = []
        for track_map in track_maps:
            trajectory = trajectory + compute_trajectory(track_map).trajectory
        final_trajectory = Trajectory(self.parameters)
        final_trajectory.set_trajectory(trajectory)
        send_trajectory_to_ros_topic(
            final_trajectory, self.trajectory_publisher, LTP_Plan)


class AutoCross(Race):
    def __init__(self, parameters: Parameters, race_state: RaceState):
        super().__init__(parameters, race_state)

    def race_loop(self):
        while self.race_state.get_finished_status() == False:
            if self.race_state.track_map_updated == True:
                track_map = self.race_state.get_track_map()
                trajectory = Trajectory(self.parameters)
                # update risk
                risk = risk_fun.compute_risk_autocross()
                send_risk_to_ros_topic(risk, self.risk_publisher, Risk)
                # TODO: In theory the ParameterServer from the KB should update the risk by subscribing to the risk topic
                self.parameters.set_risk(risk)

                # Compute the trajectory
                trajectory.compute_middle_trajectory(track_map)
                # compute the velocities
                trajectory.compute_velocities()

                # Gestione fine gara
                if self.race_state.is_last_lap():
                    trajectory.trajectory[-1].velocity = 0
                    trajectory.trajectory[-1].velocity_vector = [
                        (0, 0), (0, 0)]
                trajectory._bound_velocities()

                # send the trajectory
                send_trajectory_to_ros_topic(
                    trajectory, self.trajectory_publisher, LTP_Plan)
        self.rate.sleep()


class TrackDrive(Race):
    def __init__(self, parameters: Parameters, race_state: RaceState):
        super().__init__(parameters, race_state)

    def race_loop(self):
        rospy.loginfo("wainting for cones")
        while self.race_state.get_finished_status() == False:
            if self.race_state.is_track_map_new():
                track_map = self.race_state.get_track_map()
                trajectory = Trajectory(self.parameters)
                # update risk
                risk = risk_fun.compute_risk_trackdrive()
                send_risk_to_ros_topic(risk, self.risk_publisher, Risk)
                # TODO: In theory the ParameterServer from the KB should update the risk by subscribing to the risk topic
                self.parameters.set_risk(risk)

                rospy.loginfo(
                    f"received track map with {len(track_map.get_left_cones())} left cones and {len(track_map.get_left_cones())} right cones")

                if len(track_map.get_left_cones()) > 0 and len(track_map.get_right_cones()) > 0:
                    # Compute the trajectory
                    trajectory.compute_middle_trajectory(track_map)
                    # compute the velocities
                    trajectory.compute_velocities()

                    # Gestione fine gara
                    if self.race_state.is_last_lap() and self.race_state.is_track_map_complete():
                        trajectory.trajectory[-1].velocity = 0
                        trajectory.trajectory[-1].velocity_vector = [
                            (0, 0), (0, 0)]
                        trajectory._bound_velocities()

                    # send the trajectory
                    send_trajectory_to_ros_topic(
                        trajectory, self.trajectory_publisher, LTP_Plan)

                    rospy.loginfo(
                        f"Sent plan with {len(trajectory.get_trajectory())} steps")

            self.rate.sleep()


class TestCurve(Race):
    def __init__(self, parameters: Parameters, race_state: RaceState):
        super().__init__(parameters, race_state)

    def race_loop(self):
        race_length = self.parameters.get_track_length()
        race_width = self.parameters.get_track_width()
        intra_cone_distance = self.parameters.get_intra_cone_distance()

        # Generate the track map given the info
        track_map = TrackMap()
        track_map.load_track("./src/LTP/tests/tracks/simplecurve.json")

        # Generate the Trajectory
        trajectory = Trajectory(self.parameters)
        # set the risk to the maximum possible
        self.parameters.set_risk(risk_fun.constant(
            1, self.parameters.get_min_risk(), self.parameters.get_max_risk()))
        send_risk_to_ros_topic(self.parameters.get_risk(),
                               self.risk_publisher, Risk)

        # compute the trajectory
        trajectory.compute_middle_trajectory(track_map)
        # compute the velocities
        trajectory.compute_velocities()

        # send the trajectory
        send_trajectory_to_ros_topic(
            trajectory, self.trajectory_publisher, LTP_Plan)

        #print([planstep.position for planstep in trajectory.trajectory])
