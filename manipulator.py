'''
class for the claw of the robot
- servo driver (pwm direct)
- beam break sensor
'''
from hardware.servo_driver import ContinuousServoPWM
from hardware.beam_break_sensor import BeamBreak
import time
from gpiozero import Servo

# ---------------------------------
# old servo (continuous)
# -------------------------------
class ContinuousServoClaw:
    def __init__(self, servo_pwm_channel=2, servo_chip=0, servo_center_us=1500, beam_pin=24, beam_debounce=0.01):
        self.servo = ContinuousServoPWM(pwm_channel=servo_pwm_channel, chip=servo_chip, center_us=servo_center_us)
        # self.beam = BeamBreak(pin=beam_pin, debounce=beam_debounce)

    def close(self):
        self.servo.ccw(0.85, delta_us=100) 

    def open(self):
        self.servo.cw(0.85, delta_us=100) 

    def is_object_detected(self):
        return self.beam.beam_state()

    def wait_for_object(self):
        self.beam.wait_press()
    def end_servo(self):
        self.servo.close()

class Claw:
    def __init__(self, servo_pin=18, close_angle=10, open_angle=-80):
        # Added pulse width calibration for SG51R/SG90
        self.servo = Servo(servo_pin, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        
        self.close_angle = close_angle
        self.open_angle = open_angle

    def set_angle(self, angle):
        # Ensure value is between -1 and 1
        val = angle / 90.0
        self.servo.value = max(-1.0, min(1.0, val))

    def close(self):
        self.set_angle(self.close_angle)

    def open(self):
        self.set_angle(self.open_angle)

    def end_servo(self):
        self.servo.detach()