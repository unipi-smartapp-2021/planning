"""
    Unittest Case to check whether car and race parameters are correct
"""
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import unittest
from Parameters import Parameters

class TestParametersConstants(unittest.TestCase):

    def setUp(self) -> None:
        self.parameters = Parameters()
    
    def test_car_parameters(self) -> None:
        self.assertEqual(231.1, self.parameters.get_mass(), "Wrong Car's mass setted")
        self.assertEqual(1.45, self.parameters.get_car_width(), "Wrong Car's Width setted")
        self.assertEqual(2.5, self.parameters.get_car_length(), "Wrong Car's length setted")
        self.assertEqual(3, self.parameters.get_min_velocity(), "Wrong Car's min velocity setted")
        self.assertEqual(20, self.parameters.get_max_velocity(), "Wrong Car's max velocity setted")
        self.assertEqual(0.6, self.parameters.get_friction(), "Wrong Car's Friction setted")
        self.assertEqual(9.81, self.parameters.get_gravity(), "Wrong Car's Gravity setted")
        self.assertEqual(1360.2546, self.parameters.get_grip_force(), "Wrong Car's Grip Force setted")
        self.assertEqual(2e3, self.parameters.get_max_theoretical_forward_force(),
                         "Wrong Theoretial Forward Force setted")
        self.assertEqual(1360.2546, self.parameters.get_max_acceleration_force(), 
                         "Wrong Car's acceleration force setted")
        self.assertEqual(5.886, self.parameters.get_max_acceleration(), "Wrong Car's max accelleration setted")
        self.assertEqual(-5.886, self.parameters.get_max_deceleration(),
                         "Wrong Car's Max Deceleration setted")


    def test_race_parameters(self):
        self.assertEqual(75, self.parameters.get_track_length(), "Wrong Track's length setted")
        self.assertEqual(3, self.parameters.get_track_width(), "Wrong Track's width setted")
        self.assertEqual(5, self.parameters.get_intra_cone_distance(), "Wrong Track's intracone distance setted")
        self.assertEqual(10, self.parameters.get_num_laps(), "Wrong Race's Laps setted")

    def test_risk_condition_parameters(self):
        self.assertEqual(0.1, self.parameters.get_min_risk(), "Wrong Min Risk setting")
        self.assertEqual(0.9, self.parameters.get_max_risk(), "Wrong Max Risk setting")
        self.assertEqual(0.2, self.parameters.get_wetness(), "Wrong Wetness condition setting")
        self.assertEqual(0, self.parameters.get_fogness(), "Wrong Fogness condition setting")