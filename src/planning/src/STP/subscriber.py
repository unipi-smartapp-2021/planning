import rospy
from planning.msg import STP_Data

def callback(data):
    rospy.loginfo(f"dt: {data.dt} - dv: {data.dv}")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.Subscriber("stp_data", STP_Data, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.init_node('listener', anonymous=False)
    while not rospy.is_shutdown():
        continue

    rospy.spin()

if __name__ == '__main__':
    listener()