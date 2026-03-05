
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

class ContinuousServo:
    """
    speed in [-1, 1]
    0 = stop (trim may be needed)
    """
    def __init__(self, pin=18, min_pulse_width=0.0010, max_pulse_width=0.0020, trim=0.0):
        self.servo = Servo(
            pin,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
            pin_factory=PiGPIOFactory(),  # cleaner pulses
        )
        self.trim = float(trim)
        self.stop()

    def set_speed(self, speed: float):
        speed = max(-1.0, min(1.0, float(speed)))
        cmd = max(-1.0, min(1.0, speed + self.trim))
        self.servo.value = cmd

    def stop(self):
        self.servo.value = max(-1.0, min(1.0, 0.0 + self.trim))

    def move_approx_angle(self, degrees, speed=0.5):
        # NOTE: You will need to calibrate this multiplier!
        sec_per_degree = 0.005 
        duration = abs(degrees) * sec_per_degree
        
        # Determine direction based on degrees sign
        move_speed = speed if degrees > 0 else -speed
        
        self.set_speed(move_speed)
        time.sleep(duration)
        self.stop()

def main():
    servo = ContinuousServo(pin=18, trim=0.05)  # example pin and trim
    try:
        print("Forward")
        servo.set_speed(0.5)
        time.sleep(2)

        print("Reverse")
        servo.set_speed(-0.5)
        time.sleep(2)

        print("Stop")
        servo.stop()
        time.sleep(1)

        servo.move_approx_angle(10, speed=0.5)

    finally:
        servo.stop()

