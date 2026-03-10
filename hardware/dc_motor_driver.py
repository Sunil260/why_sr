from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep

#class to drive a dcmotor via a L298N H-bridge driver board

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
            # coast

    def stop(self, coast=True):
        self.en.value = 0
        if coast:
            # en_pin = low
            self.in1.off(); self.in2.off(); self.en.value = 0
        else:
            # both in are equal (high or low) for brake
            self.in1.on(); self.in2.on()

if __name__ == "__main__":
    #Test for DCMotorL298 and DifferentialDrive
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