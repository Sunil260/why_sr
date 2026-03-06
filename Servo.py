import time
from gpiozero import Servo

class CalibratedContinuousServo:
    """
    Continuous servo controlled by speed (open-loop) and approx angle (time-based).

    trim: added to command to make stop really stop (gpiozero value units, [-1..1])
    sec_per_deg: how long to run at a given command to rotate 1 degree (seconds/degree)
    """
    def __init__(
        self,
        pin=18,
        trim=0.2,
        min_pulse_width=0.0010,
        max_pulse_width=0.0020,
        deadband=0.02,
        sec_per_deg_fwd=0.00455,
        sec_per_deg_rev=None,
        cmd_mag=0.5,   # command magnitude used for angle moves (tune if needed)
    ):
        self.servo = Servo(pin, min_pulse_width=min_pulse_width, max_pulse_width=max_pulse_width)
        self.trim = float(trim)
        self.deadband = float(deadband)
        self.sec_per_deg_fwd = float(sec_per_deg_fwd)
        self.sec_per_deg_rev = float(sec_per_deg_rev) if sec_per_deg_rev is not None else float(sec_per_deg_fwd)
        self.cmd_mag = float(cmd_mag)
        self.stop()

    def set_speed(self, speed):
        """speed in [-1, 1]"""
        speed = max(-1.0, min(1.0, float(speed)))

        # deadband around 0 so neutral is clean
        if abs(speed) < self.deadband:
            speed = 0.0

        cmd = speed + self.trim
        cmd = max(-1.0, min(1.0, cmd))
        self.servo.value = cmd

    def stop(self):
        # neutral command (speed=0) with trim applied
        cmd = max(-1.0, min(1.0, 0.0 + self.trim))
        self.servo.value = cmd

    def move_approx_angle(self, degrees):
        """
        Open-loop “angle move” using time.
        Positive degrees uses +cmd_mag, negative uses -cmd_mag.
        """
        if degrees == 0:
            return

        if degrees > 0:
            duration = abs(degrees) * self.sec_per_deg_fwd
            self.set_speed(+self.cmd_mag)
        else:
            duration = abs(degrees) * self.sec_per_deg_rev
            self.set_speed(-self.cmd_mag)

        time.sleep(duration)
        self.stop()

if __name__ == "__main__":
    s = CalibratedContinuousServo(pin=18, trim=0.2, sec_per_deg_fwd=0.00455, cmd_mag=0.5)

    try:
        s.move_approx_angle(90)
        time.sleep(1)
        s.move_approx_angle(-90)
    finally:
        s.stop()