class VelocityRamp:
    def __init__(self, start_vel=0.0, accel_rate=0.5):
        """
        accel_rate: m/s per second (e.g., 0.5 means it takes 2 seconds to reach 1m/s)
        """
        self.current_setpoint = start_vel
        self.accel_rate = accel_rate

    def update(self, final_target, dt):
        if self.current_setpoint < final_target:
            self.current_setpoint = min(self.current_setpoint + (self.accel_rate * dt), final_target)
        elif self.current_setpoint > final_target:
            self.current_setpoint = max(self.current_setpoint - (self.accel_rate * dt), final_target)
        return self.current_setpoint
