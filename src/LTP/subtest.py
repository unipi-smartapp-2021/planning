#!/usr/bin/env python3
import rospy
from planning.msg import LTP_Plan

def Compute_Velocity():
    rospy.init_node('Compute_Velocity', anonymous=True)

    def on_stub_data(ltp_data):
        print(ltp_data)

    rospy.Subscriber("ltp_plan", LTP_Plan, on_stub_data)        
    rospy.spin()

if __name__ == '__main__':
    try:
        print("ciao")
        Compute_Velocity()
    except rospy.ROSInterruptException:
        pass