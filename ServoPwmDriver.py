import time
from rpi_hardware_pwm import HardwarePWM

class ContinuousServoPWM:
    """
    Continuous rotation servo controlled by pulse width (microseconds).

    center_us: pulse at stop (often 1500us, but yours might be 1570us)
    delta_us: how far from center to drive (bigger = faster)
    """

    PERIOD_US = 20000  # 50 Hz => 20 ms period

    def __init__(self, pwm_channel=2, chip=0, center_us=1570, hz=50, min_us=1000, max_us=2000):
        self.center_us = int(center_us)
        self.min_us = int(min_us)
        self.max_us = int(max_us)

        self.pwm = HardwarePWM(pwm_channel=pwm_channel, hz=hz, chip=chip)
        self.pwm.start(self._duty_from_us(self.center_us))  # start at stop

    def _clamp_us(self, us: int) -> int:
        return max(self.min_us, min(self.max_us, int(us)))

    def _duty_from_us(self, pulse_us: int) -> float:
        pulse_us = self._clamp_us(pulse_us)
        return (pulse_us / self.PERIOD_US) * 100.0

    def set_pulse_us(self, pulse_us: int):
        """Send an absolute pulse width in microseconds."""
        self.pwm.change_duty_cycle(self._duty_from_us(pulse_us))

    def stop(self):
        """Stop motion by continuously commanding the center pulse."""
        self.set_pulse_us(self.center_us)

    def cw(self, duration_s: float, delta_us: int = 200):
        """Rotate CW for duration, then stop. (CW/CCW depends on servo wiring/model.)"""
        self.set_pulse_us(self.center_us + abs(int(delta_us)))
        time.sleep(float(duration_s))
        self.stop()

    def ccw(self, duration_s: float, delta_us: int = 200):
        """Rotate CCW for duration, then stop."""
        self.set_pulse_us(self.center_us - abs(int(delta_us)))
        time.sleep(float(duration_s))
        self.stop()

    def close(self):
        """Hold stop briefly, then stop PWM output."""
        self.stop()
        time.sleep(0.2)   # helps prevent an exit twitch
        self.pwm.stop()


if __name__ == "__main__":
    # GPIO18 on Pi 5 commonly maps to pwm_channel=2 for this library.
    servo = ContinuousServoPWM(pwm_channel=2, chip=0, center_us=1500)

    try:
        servo.stop()
        time.sleep(1)

        servo.ccw(0.55, delta_us=100)
        time.sleep(0.5)

        servo.cw(0.55, delta_us=100) 
        time.sleep(0.5)

        servo.stop()
        time.sleep(2)

    finally:
        servo.close()