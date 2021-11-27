#!/usr/bin/python3
import rospy
import math
from planning.msg import STP_Data, LTP_Plan
from STP.Stp import STP

class RosStpNode():
    def __init__(self):
        self.stp = None
        self.actuator_pub = None

    def run(self):
        """STP component - maintain same structure of the current one
        Add methods to update informations reading from topics
        Adapt output computation to produce the correct format of data to be sent to Actuators
        """
        # Create STP object
        self.stp = STP()
        self.stp.set_car_pos_vel(((0, 0)), (0.1, 0))

        """Ros Part
        Create topic to post commands so the actuators can receive data
        Subscribe to PewDiePie but also to LTP and possibly KB to retrieve data to be used in computation
        The handler of the topics should be a method in stp that possibly updates data that is used for the computation (ie. car position, car speed etc.)
        """
        # Create publisher for Execution component
        self.actuator_pub = rospy.Publisher("stp_data", STP_Data, queue_size=10)
        # Subscribe to topics
        rospy.Subscriber("ltp_plan", LTP_Plan, self.stp.update_ltp)
        # rospy.subscribe("car_info", ... )

        rospy.init_node("stp_node", anonymous=True)

        while not rospy.is_shutdown():
            """
            STP using current stored data computes command that are published
            """
            command = self.stp.compute()
            if command is None:
                rospy.loginfo("No LTP plan, can't move.")
                self.actuator_pub.publish(0, 0)
                rospy.sleep(5)
            elif command == -1:
                rospy.loginfo("End of plan. Wtf should I do?")
                self.actuator_pub.publish(0, -math.inf)
                exit()
            else:
                # rospy.loginfo(f"dt: {command[0]} - dv: {command[1]}")
                self.actuator_pub.publish(command[0], command[1])
                new_car_x, new_car_y, rdx, rdy, plan_ref = command[2:]
                self.stp.set_car_pos_vel((new_car_x,new_car_y),(rdx,rdy))
                rospy.sleep(2)

if __name__ == '__main__':
    try:
        stp_node = RosStpNode()
        stp_node.run()
    except rospy.ROSInterruptException:
        pass