#!/usr/bin/env python3

"""
    main function for LTP
"""
from LTP.Parameters import Parameters
import rospy
import LTP.Race as rc

def main():
    #read parameters from parameter server
    parameters = Parameters()
    
    #read current race type from parameters
    race_type = parameters.get_race_type()
    
    # create race
    if race_type == 'acceleration':
        race = rc.Acceleration(parameters)
    elif race_type ==  'skidpad':
        race = rc.SkidPad(parameters)
    elif race_type == 'trackdrivess':
        race = rc.TrackDrive(parameters)
    elif race_type == 'autocross':
        race = rc.AutoCross(parameters)
    else:
        raise ValueError(f"Race type: {parameters.get_race_type()} doesn't exist.")
        
    race.race_loop()

if __name__ == '__main__':
    main()