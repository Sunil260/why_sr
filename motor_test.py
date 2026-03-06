from gpiozero import DigitalOutputDevice
from time import sleep

in1 = DigitalOutputDevice(17)
in2 = DigitalOutputDevice(27)

print("forward")
in1.on()
in2.off()
sleep(3)

print("reverse")
in1.off()
in2.on()
sleep(3)

print("stop")
in1.off()
in2.off()