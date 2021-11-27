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
    # create race
    race = rc.Acceleration(parameters)
    race.race_loop()

if __name__ == '__main__':
    main()