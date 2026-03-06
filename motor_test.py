from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep

in1 = DigitalOutputDevice(22)
in2 = DigitalOutputDevice(23)
pwm = PWMOutputDevice(13, frequency=500)

pwm.value = 1.0

print("forward")
in1.off()
in2.on()
sleep(3)


print("stop")
in1.off()
in2.off()
sleep(3)

print("reverse")
in1.on()
in2.off()
sleep(3)

pwm.value =0