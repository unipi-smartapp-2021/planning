"""
    Unittest case for Risk Functions 
"""
import unittest
from RiskFunctions import constant, merge_risks, risk_laps, risk_track

class TestRiskFunctions(unittest.TestCase):

    def setUp(self) -> None:
        self.min_risk = 0.1
        self.max_risk = 1

    def test_invalid_risks(self): 
        invalid_constant_risk = constant(0.7, 0.2, 0.5)
        another_invalid_constant_risk = constant(0.1, 0.2, 1)
        self.assertLessEqual(0.5, invalid_constant_risk, "Risk higher than the Max Risk")
        self.assertGreaterEqual(0.2, another_invalid_constant_risk, "Risk lower than Min risk")

    def test_valid_risks_laps(self):
        computed_risk = risk_laps(1, 10, self.min_risk, self.max_risk)
        self.assertGreaterEqual(computed_risk, self.min_risk,
                                "Risk Laps should be greater or equal than min risk required")
        self.assertLessEqual(computed_risk, self.max_risk,
                             "Risk laps should be less or equal than max risks required")
        risk_first_lap = risk_laps(1, 10, self.min_risk, self.max_risk)
        risk_middle_lap = risk_laps(5, 10, self.min_risk, self.max_risk)
        risk_last_lap = risk_laps(10, 10, self.min_risk, self.max_risk)
        self.assertLessEqual(risk_first_lap, risk_middle_lap,
                        "Risk at the first Lap should be less than the risk in the middle of the race")
        self.assertLessEqual(risk_middle_lap, risk_last_lap, 
                        "Risk in the middle of the race should be less than the risk at the last lap")
    
    def test_valid_raining_fogness(self):
        """
            Test Valid Raining and Fogness risks
        """
        risk_without_nothing = risk_track(0, 0, self.min_risk, self.max_risk)
        risk_with_raining = risk_track(1, 0, self.min_risk, self.max_risk)
        risk_with_fogness = risk_track(0, 1, self.min_risk, self.max_risk)
        risk_with_rain_and_fog = risk_track(1, 1, self.min_risk, self.max_risk)

        self.assertGreaterEqual(risk_without_nothing, risk_with_raining, 
                             "Risk without raining and fog should be less than the" + 
                             "risk with Rain")
        self.assertGreaterEqual(risk_without_nothing, risk_with_fogness, 
                             "Risk without raining and fog should be less than the" + 
                             "risk with Fog")
        self.assertGreaterEqual(risk_without_nothing, risk_with_rain_and_fog, 
                             "Risk without raining and fog should be less than the" + 
                             "risk with Fog and Raining")
        self.assertGreaterEqual(risk_with_raining, risk_with_rain_and_fog,
                                "Risk with only raining should be greater than the " +
                                "risk of Raining and Fogness")
        
    def test_valid_constant(self):
        valid_constant_risk = constant(0.4, 0.1, 0.6)
        another_constant_risk = constant(0.7, 0.1, 0.75)
        self.assertEqual(0.4, valid_constant_risk, "Invalid Constant Risk")
        self.assertEqual(0.7, another_constant_risk, "Invalid Constant Risk assignment")


    def test_valid_merge_risk(self):
        valid_merge_risk = merge_risks([1, 0], [0.5, 0.5])
        another_valid_merge = merge_risks([0.5, 0.6], [0.2, 0.8])
        valid_merge_without_weight = merge_risks([0.4, 0.6])
        self.assertEqual(0.5, valid_merge_risk, "Invalid Merge risk value")
        self.assertEqual(0.58, another_valid_merge, "Invalid Merge Risk value")
        self.assertEqual(0.5, valid_merge_without_weight, "Invalid Merge Risk without weight")