from typing import List, Tuple
from LTP.Trajectory import Trajectory
from planning.msg import LTP_Plan
from rospy.topics import Publisher

def send_trajectory_to_ros_topic(trajectory: Trajectory, publisher : Publisher, msg_type: LTP_Plan) -> Tuple[List[float], List[float], List[float], List[float]]:
    points_x = list(map(lambda pos: pos.position[0], trajectory.get_trajectory()))
    points_y = list(map(lambda pos: pos.position[1], trajectory.get_trajectory()))
    vel_x = list(map(lambda vel: vel.velocity_vector[0], trajectory.get_trajectory()))
    vel_y = list(map(lambda vel: vel.velocity_vector[1], trajectory.get_trajectory()))
    msg = msg_type(points_x, points_y, vel_x, vel_y)
    publisher.publish(msg)