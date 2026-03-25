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
class LineEstimate:
    detected: bool
    x_error_center: float
    x_error_ahead: float
    heading_error_ahead: float

@dataclass
class TargetEstimate:
    detected: bool
    centroid_x: float
    centroid_y: float
    error_x: float
    error_y: float
    area: float

@dataclass
class SafeZoneEstimate:
    detected: bool
    centroid_x: float
    error_x: float
    error_y: float
    area: float
 
class BaseController:
    def compute(self, estimate, dt):
        raise NotImplementedError

class PDController:
    def __init__(self, kp: float, kd: float):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0.0
        self.first_update = True

    def reset(self):
        self.prev_error = 0.0
        self.first_update = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 1e-6:
            dt = 1e-6

        if self.first_update:
            d_error = 0.0
            self.first_update = False
        else:
            d_error = (error - self.prev_error) / dt

        self.prev_error = error
        return self.kp * error + self.kd * d_error
    
    def update_params(self, kp = None, kd = None):
        if kp is not None:
            self.kp = kp
        if kd is not None:
            self.kd = kd

class LineFollowingController(BaseController):
    def __init__(self, k_heading_slow=1.0, v_min=0.25, v_max=0.5, omega_max=0.4):

        self.lateral_pd = PDController(kp=0.5, kd=0)
        self.k_heading_slow = k_heading_slow
        self.v_min = v_min
        self.v_max = v_max
        self.omega_max = omega_max

    def compute(self, estimate: LineEstimate, dt: float, base_speed: float = 0.25):
        if not estimate.detected:
            return DriveCommand(v=0.0, omega=0.0)

        lateral_error = estimate.x_error_center
        heading_error = estimate.heading_error_ahead

        omega = self.lateral_pd.compute(lateral_error, dt)
        
        v = base_speed - self.k_heading_slow * abs(heading_error)
        v = max(self.v_min, min(self.v_max, v))

        omega = max(-self.omega_max, min(self.omega_max, omega))

        return DriveCommand(v=v, omega=omega)
    
class AlignmentController(BaseController):
    def __init__(self, omega_max=0.3, x_tol=0.025): #changing from 0.025
        self.align_pd = PDController(kp=1.5, kd=0.5)
        self.omega_max = omega_max
        self.x_tol = x_tol

    def compute(self, estimate: TargetEstimate, dt: float):
        if not estimate.detected:
            return DriveCommand(v=0.0, omega=0.0)

        if abs(estimate.error_x) < self.x_tol:
            return DriveCommand(v=0.0, omega=0.0)

        omega = self.align_pd.compute(estimate.error_x, dt)
        omega = max(-self.omega_max, min(self.omega_max, omega))
        return DriveCommand(v=0.0, omega=omega)

class ApproachController(BaseController):
    def __init__(self, omega_max=0.25, v_max=0.25, pickup_y = 300, x_tol = 0.05):
        self.lateral_pd = PDController(kp=0.05, kd=0.02)
        self.Kpy = 0.2
        self.omega_max = omega_max
        self.v_max = v_max
        self.pickup_y = pickup_y
        self.x_tol = x_tol

    def compute(self, estimate: TargetEstimate, dt: float):
        if not estimate.detected:
            return DriveCommand(v=0.0, omega=0.0)

        omega = self.lateral_pd.compute(estimate.e_x, dt)
        omega = max(-self.omega_max, min(self.omega_max, omega))

        if abs(estimate.e_x) < 0.1:
            omega = 0.0

        if (estimate.centroid_y >= self.pickup_y) and (abs(estimate.e_x)<= self.x_tol):
            return DriveCommand(v=0.0, omega=0.0)
        

        # Example: smaller detected area -> farther away -> move faster
        v = self.Kpy * (self.pickup_y - estimate.centroid_y)
        v = max(0.25, min(self.v_max, v))

     
        return DriveCommand(v=v, omega=omega)
    
class TurnUntilLineController(BaseController):
    def __init__(self, search_omega=0.2):
        self.search_omega = search_omega

    def compute(self, estimate: LineEstimate, dt: float):
        if estimate.detected:
            return DriveCommand(v=0.0, omega=0.0)

        return DriveCommand(v=0.0, omega=self.search_omega)