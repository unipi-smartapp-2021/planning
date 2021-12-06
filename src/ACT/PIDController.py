import rospy
import numpy as np

# Generalize as PID controller
class PIDController():
    def __init__(self, value=0.0, Kp = 4.0, Ki = 3.0, Kd = 1.5,
            minv = 0, maxv = 1, guard=1.0, verbose=False):
        self.value = value
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.minv = minv
        self.maxv = maxv
        self.error = np.zeros(3)
        self.integral = 0.0
        self.verbose = verbose
        self.guard = guard
        # TODO: make this of fixed size by truncating it at each step
        self.outputs = np.array([self.get_value()])
        self.last_time = None
        
    def tick(self):
        pass

    def update(self, acceleration):
        pass

    def get_value(self):
        return self.value

    def _error(self, target):
        return target - self.value

    def pid_step(self, current, target):
        """ Takes as input a target state and the actual state.
            Produces an output signal

            PID: u(t) = K_p * e(t) + K_i * int(e(tau) dtau) + K_d e(t)''

            This is a discretized implementation of a PID controller, see https://en.wikipedia.org/wiki/PID_controller
        """
        if current is None or target is None:
            return self.outputs[-1]

        if self.last_time is None:
            dt = 0.1
        else:
            dt = rospy.get_time() - self.last_time
        
        dt = max(1e-4, dt)

        self.last_time = rospy.get_time()

        e = target - current
        P = self.Kp * e
        I = self.integral + self.Ki * e * dt
        D = self.Kd * (e - self.error[-1])/dt

        # anti wind-up
        I = max(-self.guard, I)
        I = min(self.guard, I)

        self.integral = I
        self.error[-1] = e
        output = P + I + D

        if self.verbose:
            rospy.loginfo('PID: P={:.3f} I={:.3f} D={:.3f}'.format(P, I, D))
            rospy.loginfo('PID: error={:.3f}'.format(e))
        
        # set upper and lower limits
        output = max(self.minv, output)
        output = min(self.maxv, output)

        self.outputs = np.append(self.outputs, output)

        return output

    def pid_loop(self, target, max_iter=300, verbose=True):
        # Initialize error vector
        self.error = np.zeros(3)

        i = 0
        # TODO: add stability condition
        while i < max_iter:
            output = self._pid_step(target)
            new_acc = self.actuate(output)

            # time.sleep(self.dt) # sleep self.dt seconds
            self.tick()
            i += 1

    def actuate(self, output):
        """ Actuate a given set of signals.
        Produces a new state
        """
        self.update(output)
        return self.get_value()


