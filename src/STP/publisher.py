#!/usr/bin/env python
import rospy
from planning.msg import STP_Data, LTP_Plan
from STP.Stp import STP

def talker():
    pub = rospy.Publisher('LTP', LTP_Plan, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(3) # 10hz
    version = 15
    x = 69
    while not rospy.is_shutdown():
        rospy.loginfo("sono Pdor figlio di Kmer")
        print(f"Sto pubblicando: version= {version}, x={x}")
        pub.publish(version,x)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass