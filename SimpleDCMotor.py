from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep

class DCMotorL298:
    def __init__(self, in1, in2, en_pwm, pwm_freq=1000):
        self.in1 = DigitalOutputDevice(in1)
        self.in2 = DigitalOutputDevice(in2)
        self.en  = PWMOutputDevice(en_pwm, frequency=pwm_freq)
        self.stop(coast=True)

    def set(self, speed): #setting speed for forward and reverse
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


class DifferentialDrive:
    #currently wired so that set(positive) = forward, set(negative) = reverse for both motors
    def __init__(self, left_motor, right_motor):
        self.L = left_motor
        self.R = right_motor

    def drive(self, left_cmd, right_cmd):
        self.L.set(left_cmd)
        self.R.set(right_cmd)

    def stop(self):
        self.L.stop()
        self.R.stop()

    
# ---- quick test ----
# if __name__ == "__main__":
#     left  = DCMotorL298(in1=17, in2=27, en_pwm=19) 
#     right = DCMotorL298(in1=22, in2=23, en_pwm=13)  

#     try:
#         left.set(-0.5); right.set(-0.5)
#         sleep(5)
#     finally:
#         left.stop()
#         right.stop()

# ---- quick test ----
if __name__ == "__main__":
    left  = DCMotorL298(in1=17, in2=27, en_pwm=19)
    right = DCMotorL298(in1=22, in2=23, en_pwm=13)

    try:
        step = 0.05
        delay = 0.1

# ramp forward
        for s in [i * step for i in range(0, int(0.8/step) + 1)]:
            left.set(s) #forward left motor
            right.set(s) #forward right motor
            sleep(delay) #duration of speed step

        sleep(1)

# ramp down 
        for s in [i * step for i in range(int(0.8/step), -1, -1)]:
            left.set(s) #forward left motor
            right.set(s) #forward right motor
            sleep(delay)

        sleep(1)

# ramp reverse
        for s in [-(i * step) for i in range(0, int(0.8/step) + 1)]:
            left.set(s) #backward left motor
            right.set(s) #backward right motor
            sleep(delay)

        sleep(1)

# ramp down
        for s in [-(i * step) for i in range(int(0.8/step), -1, -1)]:
            left.set(s) #backward left moror
            right.set(s) #backward right motor
            sleep(delay)

    finally:
        left.stop()
        right.stop()