cmake_minimum_required(VERSION 3.0.2)
project(planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  LTP_Plan.msg
  STP_Data.msg
  CarlaEgoVehicleStatus.msg
  CarlaEgoVehicleControl.msg
  Risk.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS 
  message_runtime
)

catkin_package(
 LIBRARIES planning
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  src/STP/ROS_Stp.py 
  src/STP/Stp.py 
  src/STP/publisher.py
  src/STP/subscriber.py

  src/ACT/dispatcher.py
  src/ACT/PIDController.py
  src/ACT/setup.py
  src/ACT/topics.py
  
  src/LTP/Bernstein.py
  src/LTP/GraphicUtility.py
  src/LTP/Main_LTP.py
  src/LTP/Parameters.py
  src/LTP/PlanStep.py
  src/LTP/Race.py
  src/LTP/RaceState.py
  src/LTP/RiskFunctions.py
  src/LTP/ROSInterface.py
  src/LTP/SampleTrack.py
  src/LTP/subtest.py
  src/LTP/TrackMap.py
  src/LTP/TrackOptimizer.py
  src/LTP/TrajectoryOptimizer.py
  src/LTP/Utils.py
  src/LTP/Trajectory.py

  src/SNS/...

  src/KB/...

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)