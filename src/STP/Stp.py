import numpy as np
import math
from LTP.Trajectory import Trajectory
from LTP.PlanStep import PlanStep
from LTP.Parameters import Parameters

class Car():
    ''' Car class
    This class contains all the information at current time of the car.
    Such informations are:
    - current_position_x
    - current_position_y
    - current_speed_x
    - current_speed_y
    '''
    def __init__(self):
        self.current_position_x = 0
        self.current_position_y = 0
        self.current_speed_x = 0.1
        self.current_speed_y = 0.1

    def get_position(self):
        return np.array([self.current_position_x, self.current_position_y])
    
    def set_position(self, x, y):
        self.current_position_x = x
        self.current_position_y = y
    
    def get_speed(self):
        return np.array([self.current_speed_x, self.current_speed_y])
    
    def set_speed(self, x, y):
        self.current_speed_x = x
        self.current_speed_y = y

class STP():
    def __init__(self):
        self.trajectory = None
        self.car = Car()
        self.last_plan_index = 0
    
    def callback(self,data):
        print(f"I received: {data}")

    def set_trajectory(self, t):
        self.trajectory = t

    def get_trajectory(self):
        return self.trajectory

    def set_car_pos_vel(self, position, speed):
        self.car.set_position(*position)
        self.car.set_speed(*speed)

    def get_car_pos_vel(self):
        return self.car.get_position(), self.car.get_speed()

    def rad_to_deg(self, angle):
        return angle*180/math.pi
    
    def deg_to_rad(self, angle):
        return angle*math.pi/180

    def get_direction(self, a, b):
        return a[0]-b[0], a[1]-b[1]

    def get_distance_mag(self, a, b):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

    def get_distance_comp(self, a, b):
        return (b[0]-a[0], b[1]-a[1])

    def compute_angle(self, u, v, verbose=False):
        dot_product = np.dot(u, v)
        denominator = np.linalg.norm(u)*np.linalg.norm(v)
        cosine = dot_product/denominator
        return math.acos(cosine) 

    # def KB_info(self, current_position, current_speed):
    #     ''' Get info with [Topic] from KB for car position, speed, ecc..
    #     '''
    #     self.car.set_position(*current_position)
    #     self.car.set_speed(*current_speed)

    # TODO: works only for no ltp plan update
    def update_ltp(self, data):
        p_x, p_y, v_x, v_y = data.pos_x_list, data.pos_y_list, data.vel_x_list, data.vel_y_list
        assert(len(p_x) == len(p_y) == len(v_x) == len(v_y))
        t = Trajectory(Parameters())
        new_trajectory = []
        for i in range(len(p_x)):
            p = PlanStep((p_x[i] + self.car.current_position_x, p_y[i] + self.car.current_position_y), round(math.sqrt(v_y[i]**2 + v_x[i]**2), 3), (v_x[i], v_y[i]))
            print(p)
            new_trajectory.append(p)
        t.set_trajectory(new_trajectory)
        self.set_trajectory(t)
        # TODO: new plan -> update last_plan_index variable
        # if new plan start from current position is fine
        # else find new closest point wrt old trajectory
        self.last_plan_index = 0
        
    # TODO: Works only for the acceleration scenario
    def update_car_v(self, data):
        self.car.scurrent_speed_x = data.velocity*3.6 # km/h
        # self.car.current_speed_y = data.velocity

    def rotate(self, x, y, alpha):
        #Given the x,y, return its coordinate in the rotated axis (CCW rotation angle wrt X axis)
        X = np.round(x*math.cos(alpha) + y*math.sin(alpha),5)
        Y = np.round(y*math.cos(alpha) - x*math.sin(alpha),5)
        return X, Y
    
    def polar_coordinates(self, x, y):
        theta = math.atan(y/x)
        if theta <= 0:
            theta += math.pi
        ro = math.sqrt(y**2 + x**2)
        return ro, theta

    def compute(self):
        if self.trajectory is None:
            return None
        print("-"*20 + "Computing next step" + "-"*20)
        # print("Alle curve si va dritto!")
        # 1. Find next plan point
        t = self.trajectory.get_trajectory()
        index = self.last_plan_index%len(t)
        d = math.inf
        min_index = index
        first = True
        while True:
            if index == self.last_plan_index:
                if not first:
                    print("Loop all plan and no point was found.")
                    return -1
                else:
                    first = False
            curr_p = t[index]
            dist = self.get_distance_mag(self.car.get_position(), curr_p.position)
            dist_c = self.get_distance_comp(self.car.get_position(), curr_p.position)
            psi = self.compute_angle(self.car.get_speed(), dist_c)
            print(f"index: {index} - car_pos: {self.car.get_position()}, plan_pos: {curr_p.position}, distance: {dist}, psi: {math.degrees(psi)}, cur_min_d: {d}")
            if psi >= math.pi/2: # discard point behind
                index = (index+1)%len(t)
                continue
            if dist < d: # closest point in front of me
                d = dist
                min_index = index
                index = (index+1)%len(t)
            else:
                break

        self.last_plan_index = min_index
        # print(min_index, d)
        # 2. Compute next components
        # car   (v_x, v_y)
        # plan  (v_x, v_y)
        print(f"car_pos: {self.car.get_position()}, car_v: {self.car.get_speed()}")
        print(f"plan_pos: {t[min_index].position}, plan_v: {t[min_index].velocity_vector}, plan_index: {min_index}")
        dist = self.get_distance_comp(self.car.get_position(), t[min_index].position)
        print(f"dist_comp: {dist}")
        # math.atan2(y,x) != math.atan(y/x) per elementi nel secondo-terzo quadrante
        # theta_1 = math.atan2(*np.flip(self.car.get_speed()))
        # print(self.car.current_speed_y/self.car.current_speed_x)

        #theta is the angle between car_speed and the axis origin.
        theta = math.atan(self.car.current_speed_y/self.car.current_speed_x)
        
        # print(f"Theta (°): {math.degrees(theta_1)}")
        print(f"Theta (°): {math.degrees(theta)}")
        alpha = 0
        if self.car.get_speed()[0] >= 0: #Check if the x component of car speed is positive 
            alpha = 3*math.pi/2 + theta
        else:
            alpha = math.pi/2 + theta
            
        print(f"Alpha (°): {math.degrees(alpha)}")
        car_v_r = self.rotate(*self.car.get_speed(), alpha)
        plan_v_r = self.rotate(*t[min_index].velocity_vector, alpha)
        dist_r = self.rotate(*dist, alpha)
        print(f"car_v_r: {car_v_r}")
        print(f"plan_v_r: {plan_v_r}")
        optimal = np.add(dist_r, plan_v_r)
        print(f"optimal: {optimal}")        
        real = np.add(optimal, car_v_r)
        print(f"real: {real}")
        real_module, real_angle = self.polar_coordinates(*real)
        car_v_r_module, car_v_r_angle = self.polar_coordinates(*car_v_r)
        print(f"real_angle: {math.degrees(real_angle)}, real_module: {real_module}")
        print(f"car_v_r_angle: {math.degrees(car_v_r_angle)}, car_v_r_module: {car_v_r_module}")
        delta_theta = car_v_r_angle - real_angle
        print(f"delta_theta: {math.degrees(delta_theta)}") # degrees wrt y-car-axis 
        delta_v = t[min_index].velocity - car_v_r_module
        print(f"delta_v: {delta_v}")
        
        # define simulation
        new_car_module = car_v_r_module + delta_v/2
        print(f"new_car_module : {new_car_module}")
        new_car_angle = car_v_r_angle - delta_theta
        print(f"new_car_angle : {math.degrees(new_car_angle)}")
        d_vx = new_car_module * math.cos(new_car_angle)
        d_vy = new_car_module * math.sin(new_car_angle)
        print(f"dx: {d_vx}, dy: {d_vy}")
        rdx, rdy = self.rotate(d_vx, d_vy, -alpha)
        print(f"rdx: {rdx}, rdy: {rdy}")
        new_car_x = self.car.current_position_x + rdx
        new_car_y = self.car.current_position_y + rdy
        print(f"car_x: {self.car.current_position_x}, car_y: {self.car.current_position_y}")
        print(f"new_car_x : {new_car_x}, new_car_y : {new_car_y}")

        return delta_theta, delta_v, new_car_x, new_car_y, rdx, rdy, t[min_index].position

    def __str__(self):
        return "tomareomo"