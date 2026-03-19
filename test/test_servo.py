from gpiozero import Servo
import time


servo_test = Servo(18, min_pulse_width=0.5/1000, max_pulse_width =2.5/1000)

try:

    servo_test.min()
    
    time.sleep(1)

    servo_test.mid()
    time.sleep(1)

    servo_test.max()
    time.sleep(1)
finally:

    pass