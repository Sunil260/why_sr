from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep

class DCMotorL298:
    def __init__(self, in1, in2, en_pwm, pwm_freq=20000):
        self.in1 = DigitalOutputDevice(in1)
        self.in2 = DigitalOutputDevice(in2)
        self.en  = PWMOutputDevice(en_pwm, frequency=pwm_freq)
        self.stop(coast=True)

    def set(self, speed):
        """
        speed in [-1.0, 1.0]
        + = forward, - = reverse
        """
        speed = max(-1.0, min(1.0, float(speed)))

        if speed > 0:
            self.in1.on(); self.in2.off()
            self.en.value = speed
        elif speed < 0:
            self.in1.off(); self.in2.on()
            self.en.value = -speed
        else:
            self.en.value = 0
            # keep direction pins as-is (or call stop)

    def stop(self, coast=True):
        self.en.value = 0
        if coast:
            # both low = coast on many H-bridges
            self.in1.off(); self.in2.off()
        else:
            # brake (often both high; depends on driver/board)
            self.in1.on(); self.in2.on()

# ---- quick test ----
if __name__ == "__main__":
    left  = DCMotorL298(in1=17, in2=27, en_pwm=18)  # example pins
    right = DCMotorL298(in1=22, in2=23, en_pwm=13)  # example pins

    try:
        left.set(0.3); right.set(0.3)
        sleep(2)
        left.set(0.6); right.set(0.6)
        sleep(2)
        left.set(0.0); right.set(0.0)
        sleep(1)
        left.set(-0.3); right.set(-0.3)
        sleep(2)
    finally:
        left.stop()
        right.stop()