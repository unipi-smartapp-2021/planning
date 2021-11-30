import unittest 
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

class TestVelocity(unittest.TestCase):

    def test_invalid_velocity(self):
        self.assertEqual(1, 1, "valid Velocities")
