from abc import ABC, abstractmethod

class TrackOptimizer:
    """
    a class handling the trajectory optimization from a middle circuit trajectory
    to a riskier one able to cut the circuit to achieve better performance
    """

    def __init__(self, trajectory):
        self.trajectory = trajectory 

    @abstractmethod
    def noofsides(self):
        pass
