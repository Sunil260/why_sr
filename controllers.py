''' 
Controller implementations for 2wd s&r why robot

- Path tracker PD controller + Turn Velocity controller
- Allignment controller (for the lego man pickup and drop off)
- Target approach controller
- Turn until line controller (for the 180 turn and the safe zone exit)

- no velocity control on the motors just pwm + maybe a manual trim to balance both sides

'''
from dataclasses import dataclass


@dataclass
class DriveCommand:
    v: float
    omega: float

@dataclass 
class SafeZoneEstimate:
    detected: bool
    centroid_x: float
    error_x: float
    error_y: float
    area: float
    # confidence: float

@dataclass
class TargetEstimate:
    detected: bool
    centroid_x: float
    centroid_y: float
    error_x: float
    error_y: float
    area: float
    # distance_error: float
    # confidence: float
    
@dataclass
class LineEstimate:
    detected: bool
    x_error_center: float #px
    x_error_ahead: float #px
    heading_error_ahead: float #rads
    # confidence: float


class BaseController:
    def compute(self, estimate, dt):
        raise NotImplementedError

class LineFollowingController(BaseController):
    def __init__(self):
        pass

    def compute(self, heading_error, lateral_error):
       
       # simple PD controller for line following
       # the perception module -> x_error_center: float #px, x_error_ahead: float #px, heading_error_ahead: float #rads
       # 
    
        Kp_heading = 1.0
        Kd_heading = 0.1

        Kp_lateral = 0.5
        Kd_lateral = 0.05

        pass

class AlignmentController(BaseController):
    def __init__(self):
        pass

    def compute(self, target_estimate, dt):
        # simple P controller to align with the target (lego
        pass

class ApproachController(BaseController):
    def __init__(self):
        pass

    def compute(self, target_estimate, dt):
        # simple P controller to approach the target
        pass

class TurnUntilLineController(BaseController):
    def __init__(self):
        pass

    def compute(self, line_estimate, dt):
        # simple P controller to turn until the line is detected
        pass