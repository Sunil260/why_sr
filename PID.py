
import time
# from gpiozero

# PID Controller Class
class PID:
    def __init__(self, kp, ki, kd, min_out=0.0, max_out=1.0, name="motor"):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_out, self.max_out = min_out, max_out
        self.prev_error = 0
        self.integral = 0
        self.name = name

    def compute(self, setpoint, actual, dt):
        error = setpoint - actual
        self.integral += error * dt
        # Anti-windup: limit integral to your max output range
        self.integral = max(min(self.integral, self.max_out), self.min_out)
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output
    
    def update_params(self, kp=None, ki=None, kd=None):
        if kp is not None: self.kp = float(kp)
        if ki is not None: self.ki = float(ki)
        if kd is not None: self.kd = float(kd)
