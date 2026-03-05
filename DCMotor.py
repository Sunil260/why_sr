''' DC Motor Class
    -used to instantiate a DC motor object with specified GPIO pins
    -provides methods to set motor speed and direction using PWM signals
    -optional encoder feedback for closed loop control
    
'''

# from gpiozero import Motor, RotaryEncoder

# class DCMotor:
#     def __init__(self, forward, backward, pwm_pin, enc_a, enc_b, wheel_circ=0.159, cpr=700):
#         self.motor = Motor(forward=forward, backward=backward, enable=pwm_pin)
#         self.encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)
#         self.prev_ticks = 0
#         self.ticks_per_m = cpr / wheel_circ # ~4402.5 ticks per meter

#     def get_velocity_ms(self, dt):
#         """Returns velocity in Meters Per Second"""
#         current_ticks = self.encoder.steps
#         delta_ticks = current_ticks - self.prev_ticks
#         self.prev_ticks = current_ticks
        
#         # (ticks / dt) = ticks per second. Divide by ticks_per_m to get m/s.
#         velocity_ms = (delta_ticks / dt) / self.ticks_per_m
#         return velocity_ms

#     def set_speed(self, speed):
#         # speed is PWM duty cycle 0.0 to 1.0
#         self.motor.forward(max(0.0, min(speed, 1.0)))

#     def stop(self):
#         self.motor.stop()
from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder

class DCMotorL298:
    def __init__(self, in1, in2, ena_pwm, enc_a, enc_b, wheel_circ_m, counts_per_rev):
        self.in1 = DigitalOutputDevice(in1)
        self.in2 = DigitalOutputDevice(in2)
        self.ena = PWMOutputDevice(ena_pwm, frequency=20000)  # quieter than 100 Hz
        self.encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)

        self.prev_ticks = self.encoder.steps
        self.ticks_per_m = counts_per_rev / wheel_circ_m

    def set_speed(self, speed):
        # speed in [-1, 1]
        speed = max(-1.0, min(1.0, float(speed)))

        if speed > 0:
            self.in1.on(); self.in2.off()
            self.ena.value = speed
        elif speed < 0:
            self.in1.off(); self.in2.on()
            self.ena.value = -speed
        else:
            self.ena.value = 0
            # coast:
            self.in1.off(); self.in2.off()

    def get_velocity_ms(self, dt):
        if dt <= 0:
            return 0.0
        ticks = self.encoder.steps
        delta = ticks - self.prev_ticks
        self.prev_ticks = ticks
        return (delta / dt) / self.ticks_per_m