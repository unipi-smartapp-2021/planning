from LTP.TrackMap import TrackMap

TRACK_MAP_TOPIC = 'track_map'
FINISHED_FLAG_TOPIC = 'finished'
CURRENT_LAP_TOPIC = 'current_lap'


class RaceState():

    def __init__(self, subscribe=False):
        """Constructor for RaceState object

        Args:
            subscribe (bool, optional): If True, the RaceState will automatically 
                subscribe to the topics using the default callbacks. 
                If False, you have to use the subscribe methods. Defaults to False.
        """
        
        # TODO subscribe to get map
        # TODO subscribe to get current state of the race (finisched or not)
        # TODO subscribe to get current lap
        self.trackMap = TrackMap()
        self.trackMap.load_track('tests/tracks/TarascoRace.json')
        self.finished = False
        self.current_lap = 1

    def update_track_map(self):
        """callback used when new map arrives
        """
        pass

    def subscribe_track_map(self, callback: function):
        """subscribes to track map topic

        Args:
            callback (function): callback called when new value arrive for the topic
        """
        pass
    
    def unsubscribe_track_map(self):
        """unsubscribe to track map topic
        """

    def get_track_map(self):
        """returns last trackmap read from KB

        Returns:
            TrackMap: last trackmap read from KB
        """
        return self.trackMap

    def update_finished(self):
        """callback used when new finisched status arrives
        """
        pass

    def subscribe_finisched_flag(self, callback: function):
        """subscribes to finisched flag topic

        Args:
            callback (function): callback called when new value arrive for the topic
        """
        pass
    
    def unsubscribe_finisched_flag(self):
        """unsubscribe from finisched flag topic
        """
        pass

    def get_finished_status(self):
        """returns current status of finisched flag

        Returns:
            boolean: current status of finisched flag
        """
        return self.finished

    def subscribe_current_lap(self, callback: function):
        """subscribes to current lap flag topic

        Args:
            callback (function): callback called when new value arrive for the topic
        """
        pass
    
    def unsubscribe_current_lap(self):
        """unsubscribe from current_lap topic
        """
        pass
    
    def update_current_lap(self):
        """update current lap
        """
        pass

    def get_current_lap(self):
        """returns

        Returns:
            [type]: [description]
        """
        return self.current_lap
