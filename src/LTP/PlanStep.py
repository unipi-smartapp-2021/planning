"""
    Represent a Step of our Car Planning
"""
from typing import Tuple, List
import math 

class PlanStep:
    """
        Define a Step of a Plan which consist to a tuple of position (x, y)
        and the desired car velocity
    """
    def __init__(self, position: Tuple[float, float], velocity: float = 0, velocity_vector: List[Tuple[float, float]] = [0, 0]):
        self.position = position

        if (velocity < 0):
            raise ValueError("Velocity must be a positive value")
        self.velocity = velocity
        self.velocity_vector = []

    def get_position(self):
        """
            Return (x, y) position of the PlanStep 
        """
        return self.position

    def get_velocity(self):
        """
            Return velocity of the step of the Plan
        """
        return self.velocity

    def __str__(self):
        return "PlanStep(position={}, velocity={}, velocity_vector={})".format(self.position, self.velocity, self.velocity_vector)