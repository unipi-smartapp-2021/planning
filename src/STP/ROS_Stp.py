#!/usr/bin/python3
import rospy
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

        """Ros Part
        Create topic to post commands so the actuators can receive data
        Subscribe to PewDiePie but also to LTP and possibly KB to retrieve data to be used in computation
        The handler of the topics should be a method in stp that possibly updates data that is used for the computation (ie. car position, car speed etc.)
        """
        rospy.init_node("stp_node", anonymous=True)
        # Create publisher for Execution component
        self.actuator_pub = rospy.Publisher("stp_data", STP_Data)
        # Subscribe to topics
        rospy.Subscriber("ltp_plan", LTP_Plan, self.stp.update_ltp)
        # rospy.subscribe("car_info", ... )

        while not rospy.is_shutdown():
            """
            STP using current stored data computes command that are published
            """
            command = self.stp.compute()
            if command is None:
                print("No LTP plan, can't move.")
            else:
                self.actuator_pub.publish(command)
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        stp_node = RosStpNode()
        stp_node.run()
    except rospy.ROSInterruptException:
        pass