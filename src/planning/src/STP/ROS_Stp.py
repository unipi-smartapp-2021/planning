#!/usr/bin/python3
import rospy
import math
import carla
import matplotlib.pyplot as plt
from planning.msg import STP_Data, LTP_Plan, CarlaEgoVehicleStatus
from STP.Stp import STP
from LTP.TrackMap import TrackMap

class RosStpNode():
    def __init__(self):
        self.stp = None
        self.actuator_pub = None
        self.trackMap= TrackMap()
        self.trackMap.load_track("./src/LTP/tests/tracks/acceleration.json")
        self.cones_offset_x = 0
        self.cones_offset_y = 0
        
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.actor_list = self.world.get_actors()
        self.kerubless = self.actor_list.filter("vehicle.eteam.kerubless")[0]

    def print(self,ltp_plan, new_figure=False):
        if new_figure is True:
            plt.figure()
            plt.title('Map')
            plt.xlabel('Position x')
            plt.ylabel('Position y')
        
        plt.scatter([pos_x + self.cones_offset_x for pos_x, _ in self.trackMap.left_cones], [pos_y + self.cones_offset_y for _, pos_y in self.trackMap.left_cones],
                    color='blue', label='Left Cones')
        plt.scatter([pos_x + self.cones_offset_x for pos_x, _ in self.trackMap.right_cones], [pos_y + self.cones_offset_y for _, pos_y in self.trackMap.right_cones],
                    color='yellow', label='Right Cones')
        if ltp_plan is not None:
            plt.scatter(ltp_plan[0], ltp_plan[1], color='black', label='LTP')
        if self.trackMap.car_position is not None:
            plt.scatter(self.trackMap.car_position[0], self.trackMap.car_position[1], color='red', label='Car Pos')
        plt.show()

    def read_car_location(self):
        self.client.set_timeout(10.0)
        vehicle_x = self.kerubless.get_transform().location.x
        vehicle_y = self.kerubless.get_transform().location.y

        return vehicle_x, vehicle_y

    def run(self):
        """STP component - maintain same structure of the current one
        Add methods to update informations reading from topics
        Adapt output computation to produce the correct format of data to be sent to Actuators
        """
        # Create STP object
        self.stp = STP()
        init_car_pos = self.read_car_location()
        self.cones_offset_x, self.cones_offset_y = init_car_pos
        print(init_car_pos)
        self.trackMap.set_car_position(init_car_pos)
        self.stp.set_car_pos_vel(init_car_pos, (0, 0))

        """Ros Part
        Create topic to post commands so the actuators can receive data
        Subscribe to PewDiePie but also to LTP and possibly KB to retrieve data to be used in computation
        The handler of the topics should be a method in stp that possibly updates data that is used for the computation (ie. car position, car speed etc.)
        """
        # Create publisher for Execution component
        self.actuator_pub = rospy.Publisher("stp_data", STP_Data, queue_size=10)
        # Subscribe to topics
        rospy.Subscriber("ltp_plan", LTP_Plan, self.stp.update_ltp)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.stp.update_car_v)
        # rospy.subscribe("car_info", ... )

        rospy.init_node("stp_node", anonymous=True)
        rate = rospy.Rate(5)
        self.print(None,True) # TODO: test correct print
        while not rospy.is_shutdown():
            """
            STP using current stored data computes command that are published
            """
            command = self.stp.compute()
            if command is None:
                rospy.loginfo("No LTP plan, can't move.")
                self.actuator_pub.publish(0, 0, 0)
            elif command == -1:
                rospy.loginfo("End of plan. Wtf should I do?")
                self.actuator_pub.publish(1, 0, -math.inf)
                exit()
            elif command == -2:
                rospy.loginfo("Enter final stop state.")
                self.actuator_pub.publish(1, 0, -math.inf)
                exit()
            else:
                # rospy.loginfo(f"dt: {command[0]} - dv: {command[1]}")               
                self.actuator_pub.publish(0, command[0], command[1])
                new_car_x, new_car_y, rdx, rdy, plan_ref = command[2:]
                self.stp.car.set_position(*self.read_car_location())
                self.trackMap.set_car_position((new_car_x,new_car_y))
                # self.print(plan_ref)
            rate.sleep()

if __name__ == '__main__':
    try:
        stp_node = RosStpNode()
        stp_node.run()
    except rospy.ROSInterruptException:
        pass