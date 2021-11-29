from Race import Acceleration, AutoCross, SkidPad, TrackDrive, Race
from Parameters import Parameters
from RaceState import RaceState

def main():
    # Read data from Parameters
    parameters = Parameters()
    # Create the Race class that subscribes to the TrackMap & others topics
    race_state = RaceState()
    
    race = None

    if parameters.get_race_type() == "acceleration":
        race = Acceleration(parameters)
    elif parameters.get_race_type() == "skidpad":
        race = SkidPad(parameters)
    elif parameters.get_race_type() == "trackdrive":
        race = TrackDrive(parameters, race_state)
    elif parameters.get_race_type() == "autocross":
        race = AutoCross(parameters, race_state)
    else: 
        raise ValueError(f"Race type: {parameters.get_race_type()} doesn't exist.")
    
    try:
        race.race_loop()
    except:
        print('Ops')
        #TODO EBS()


if __name__ == '__main__':
    main()