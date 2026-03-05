

import time
from gpiozero import Servo

class ContinuousServo:
    def __init__(self, pin=18, min_pulse_width=0.0010, max_pulse_width=0.0020, trim=0.0):
        # REMOVED: PiGPIOFactory()
        self.servo = Servo(
            pin,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width
            # Default factory handles Pi 5 automatically
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
        sec_per_degree = 0.005 # Calibrate this!
        duration = abs(degrees) * sec_per_degree
        move_speed = speed if degrees > 0 else -speed
        self.set_speed(move_speed)
        time.sleep(duration)
        self.stop()

def main():
    # You do NOT need to run 'sudo pigpiod' for this version
    servo = ContinuousServo(pin=18, trim=0.05) 
    try:
        print("Moving...")
        servo.move_approx_angle(90, speed=0.5)
        time.sleep(1)
        servo.stop()
    finally:
        servo.stop()

if __name__ == "__main__":
    main()
