"""
    Define risk functions
"""

# TODO: Implement me..
def compute_risk_acceleration() -> float:
    pass

def compute_risk_trackdrive() -> float:
    pass

def compute_risk_autocross() -> float:
    pass

def compute_risk_skidpad() -> float:
    pass

def risk_laps(current_lap: int, num_laps: int, min_risk: float, max_risk: float):
    """
        Compute Risk based on number of laps
        
        Params:
            -current_lap(int): number of current lap
            -num_laps(int): number of laps of the race 
            -min_risk(float): minimum risk  
            -max_risk(float): maximum risk
    """
    return current_lap/num_laps * (max_risk - min_risk) + min_risk

def risk_track(wetness: float, fogness: float, min_risk: float, max_risk: float):
    """
        Compute risk based on wetness and fogness condition

        Params:
            -wetness(float): value of wetness of the track
            -fogness(float): value of fogness of the track
            -min_risk(float): minimum risk
            -max_risk(float): maximum risk
    """
    return (1 - (wetness + fogness) / 2) * (max_risk - min_risk) + min_risk

def constant(risk: float, min_risk: float, max_risk: float):
    """
        Constant risk between min and max risk

        Param:
            -risk(float): risk of 
            -min_risk(float): minimum risk assumed
            -max_risk(float): maximum risk that can be assumed
    """
    return max(min_risk, min(risk, max_risk))

def merge_risks(risk_values, risk_weights = None):
    """
        Merge Risks given values and weights

        Params:
            -risk_values(List): List of risk values 
            -risk_weights(List): List of risk weights 
    """
    if risk_weights == None or len(risk_weights) == 0:
        uniform_weight = 1 / len(risk_values)
        risk_weights = [uniform_weight] * len(risk_values)
    return sum([risk * weight for risk, weight in zip(risk_values, risk_weights)])