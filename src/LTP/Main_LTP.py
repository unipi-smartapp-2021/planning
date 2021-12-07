#!/usr/bin/env python3

"""
    main function for LTP
"""
from LTP.Parameters import Parameters
from LTP.RaceState import RaceState
import rospy
import LTP.Race as rc

def main():
    #read parameters from parameter server
    parameters = Parameters()
    race_state = RaceState()
    
    #read current race type from parameters
    race_type = parameters.get_race_type()

    # TODO: just for test.
    race_type = 'simplecurve'
    
    # create race
    # TODO: subscribe to the EBS topic, when reached stops sending plan
    if race_type == 'acceleration':
        race = rc.Acceleration(parameters, race_state)
    elif race_type ==  'skidpad':
        race = rc.SkidPad(parameters, race_state)
    elif race_type == 'trackdrive':
        race = rc.TrackDrive(parameters, race_state)
    elif race_type == 'autocross':
        race = rc.AutoCross(parameters, race_state)
    elif race_type == 'simplecurve':
        race = rc.TestCurve(parameters, race_state)
    else:
        raise ValueError(f"Race type: {parameters.get_race_type()} doesn't exist.")
        
    race.race_loop()

if __name__ == '__main__':
    main()