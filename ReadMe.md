# Planning v.1.0.0
First version of planning module.

## What does it plan?
In this scenario the planning module reads and reconstruct the `official acceleration track according to FSG rules` and compute a basic trajectory to complete the track.

## How it works?
- `Main_LTP.py`: This module reads the cones position in the track and computes the middle trajectory for this competition providing position of plan points and desired acceleration. <br>
The ROS node publishes on the topic `ltp_plan` the plan using the `LPT_Plan.msg`. (In this scenario the plan does not changed so is published only once)

- `ROS_Stp.py`: This module given the plan computed from the LTP module and current state of the car (*position*, *velocity*) computes the delta for the velocity and the steering angle. <br>
The ROS node subscribes to the following topics: `ltp_plan`, `/carla/ego_vehicle/vehicle_status`. The first one is used to read the long term plan, while the second one is provided by the simulator and is exploited to read the current speed of the car. As far as concerns the car position, since no map is provided, the car position is retrieved from an instance of the car object in the simulator (*CHEATING*). <br>
The node also publishes the short term plan in the topic `stp_data` using the `STP_Data.msg` message. It provides 3 values: <ul>
    <li><em>status</em>: racing (0), stop (1)</li>
    <li><em>dt</em>: delta theta = variation steering angle</li>
    <li><em>dv</em>: delta velocity = variation wrt to current speed</li></ul>

## How to run
Following the next steps is possible to run an instance of the planning module that interfaces with the Carla simulator.
1. Start the simulator
2. Start the `ROS_Stp.py` node
3. Start the `Main_LTP.py` node
 
Upon starting the ROS_Stp node will start reading the car speed and print some log info and send information on its topic.<br>
Once the Main_LTP node sends the long term plan on the topic the ROS_Stp node will start computing the actual commands with a `5Hz rate`. The output will remain the same until some changes to the car status are made.<br>
In this scenario the car does not move by itself, but moving the car with manual drive it is possible to see how the log information of the STP module adapts wrt current car position and speed.