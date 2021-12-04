import rospy
import numpy as np

# Generalize as PID controller
class PIDController():
    def __init__(self, value=0.0, Kp = 4.0, Ki = 3.0, Kd = 1.5, minv = 0, maxv = 1):
        self.value = value
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.minv = minv
        self.maxv = maxv
        self.error = np.zeros(3)
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

        A0 = self.Kp + self.Ki*dt + self.Kd/dt
        A1 = -self.Kp - 2*self.Kd/dt
        A2 = self.Kd/dt

        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = target - current
        output = self.outputs[-1] + A0 * self.error[0] + A1 * self.error[1] + A2 * self.error[2]
        self.outputs = np.append(self.outputs, output)

        # set upper and lower limits
        output = max(self.minv, output)
        output = min(self.maxv, output)
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


