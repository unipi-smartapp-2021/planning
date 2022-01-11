# parameters contains the parameters for the race

#TODO consider update of these parameters (think failures)
#TODO FORSE mettere metodo set per ogni parametro

class Parameters():
    def __init__(self):
        # get parameters from knowledgebase (TODO)
        self.RACE_TYPE = "trackdrive"  # type of race

        self.MASS = 231.1  # kg

        self.CAR_WIDTH = 1.45  # m
        self.CAR_LENGTH = 2.5  # m

        self.MIN_VELOCITY = 3  # [m/s] -> ~10 km/h
        self.MAX_VELOCITY = 20  # [m/s] -> 72 km/h

        self.FRICTION = 0.6  # God help us.

        self.GRAVITY = 9.81  # [m/s^2]

        # [N] (0.6 * 231.1 * 9.81 = 1360,2546 [N])
        self.F_GRIP = self.FRICTION * self.MASS * self.GRAVITY

        self.MAX_THEORETICAL_FORWARD_FORCE = 2e3  # [N]

        # We take the minimum between the theoretical maximum forward force
        # and the maximum supported force by the grip of the tires.
        self.MAX_ACCELERATION_FORCE = min(
            self.MAX_THEORETICAL_FORWARD_FORCE, self.F_GRIP)  # [N]

        self.MAX_ACCELERATION = self.MAX_ACCELERATION_FORCE / \
            self.MASS  # [m/s^2]

        # [m/s^2] (We assume that our breaks can generate this much deceleration)
        self.MAX_DECELERATION = -self.MAX_ACCELERATION

        self.MIN_RISK = 0.1  # minimum risk for the plan

        self.MAX_RISK = 0.9  # maximum risk for the plan

        self.FOGNESS = 0  # fog = 1 (Bergamo/Milano), fog = 0 (Sahara)

        self.WETNESS = 0.2  # the closer to 1, the more the track is wet

        # length of the race in meters (only for acceleration)
        self.ACC_TRACK_ACC_LENGTH = 75 #length of the acceleration phase in acceleration race
        self.ACC_TRACK_DEC_LENGTH = 100 #length of the deceleration phase in acceleration race

        self.TRACK_WIDTH = 3  # width of the track

        self.INTRA_CONE_DISTANCE = 5  # distance between cones

        self.NUM_LAPS = 10  # number laps of the race

        self.risk = self.MIN_RISK
        self._update_trajectory_width()
        self._update_max_velocity()

    # RISK methods

    def _update_trajectory_width(self):
        """
            update maximum trajectory width (Distance from the center) (for risk)
        """
        self.trajectory_width_risk = self.risk * self.get_track_width()

    def _update_max_velocity(self):
        """
           update maximum velocity of the track (for risk)
        """
        self.max_velocity_risk = self.risk * \
            (self.get_max_velocity() - self.get_min_velocity()) + \
            self.get_min_velocity()

    def get_risk(self):
        """
            return current risk
        """
        return self.risk

    def set_risk(self, risk):
        """
            update the risk and its corresponding parameters
        """
        self.risk = risk
        self._update_trajectory_width()
        self._update_max_velocity()

    def get_max_velocity_risk(self):
        return self.max_velocity_risk

    def get_trajectory_width_risk(self):
        return self.trajectory_width_risk

    # Generate getters and setters for all the parameters

    def get_race_type(self):
        return self.RACE_TYPE

    def get_max_risk(self):
        return self.MAX_RISK

    def get_min_risk(self):
        return self.MIN_RISK

    def get_wetness(self):
        return self.WETNESS

    def get_fogness(self):
        return self.FOGNESS

    def get_mass(self):
        return self.MASS

    def get_car_width(self):
        return self.CAR_WIDTH

    def get_car_length(self):
        return self.CAR_LENGTH

    def get_min_velocity(self):
        return self.MIN_VELOCITY

    def get_max_velocity(self):
        return self.MAX_VELOCITY

    def get_friction(self):
        return self.FRICTION

    def get_gravity(self):
        return self.GRAVITY

    def get_grip_force(self):
        return self.F_GRIP

    def get_max_theoretical_forward_force(self):
        return self.MAX_THEORETICAL_FORWARD_FORCE

    def get_max_acceleration_force(self):
        return self.MAX_ACCELERATION_FORCE

    def get_max_acceleration(self):
        return self.MAX_ACCELERATION

    def get_max_deceleration(self):
        return self.MAX_DECELERATION

    def get_acc_track_acc_length(self):
        return self.ACC_TRACK_ACC_LENGTH
    
    def get_acc_track_dec_length(self):
        return self.ACC_TRACK_DEC_LENGTH

    def get_track_width(self):
        return self.TRACK_WIDTH

    def get_intra_cone_distance(self):
        return self.INTRA_CONE_DISTANCE

    def get_num_laps(self):
        return self.NUM_LAPS