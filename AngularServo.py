import time
from gpiozero import Servo

class ContinuousServo:
    def __init__(self, pin=18, min_pulse_width=0.0010, max_pulse_width=0.0020, trim=0.0, auto_trim=True):
        self.servo = Servo(
            pin,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width
        )
        self.trim = float(trim)
        if auto_trim:
            self.calibrate_trim()
        self.stop()

    def set_speed(self, speed: float):
        # Clamp speed and add trim
        speed = max(-1.0, min(1.0, float(speed)))
        cmd = max(-1.0, min(1.0, speed + self.trim))
        self.servo.value = cmd

    def stop(self):
        self.servo.value = max(-1.0, min(1.0, 0.0 + self.trim))

    def move_approx_angle(self, degrees, speed=0.5):
        sec_per_degree = 0.005  # tune for your servo
        duration = abs(degrees) * sec_per_degree
        move_speed = speed if degrees > 0 else -speed
        self.set_speed(move_speed)
        time.sleep(duration)
        self.stop()

    def calibrate_trim(self):
        """
        Slowly sweep servo around 0 to find the true neutral pulse.
        Watch the servo: when it completely stops, note that value.
        """
        print("Calibrating trim. Watch the servo carefully...")
        test_values = [-1.5 -1.4 -1.3 -1.2 -1.1 -1.  -0.9 -0.8 -0.7 -0.6 -0.5 -0.4 -0.3 -0.2 -0.1
  0.   0.1  0.2  0.3  0.4  0.5  0.6  0.7  0.8  0.9  1.   1.1  1.2  1.3  1.4  1.5]
        for val in test_values:
            self.servo.value = val
            print(f"Testing trim value: {val}")
            time.sleep(1)

        print("Adjust self.trim manually to the value where servo stopped.")
        # Optionally, you can let user input the correct trim:
        try:
            user_trim = float(input("Enter trim value where servo stops: "))
            self.trim = user_trim
            print(f"Trim set to: {self.trim}")
        except:
            print("No input given. Using default trim.")

def main():
    servo = ContinuousServo(pin=18, auto_trim=True)
    try:
        print("Moving forward 90°...")
        servo.move_approx_angle(90, speed=0.5)
        time.sleep(1)
        print("Moving backward 90°...")
        servo.move_approx_angle(-90, speed=0.5)
        time.sleep(1)
        servo.stop()
    finally:
        servo.stop()

if __name__ == "__main__":
    main()