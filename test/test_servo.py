# from gpiozero import Servo
# import time


# servo_test = Servo(18, min_pulse_width=0.5/1000, max_pulse_width =2.5/1000)

# try:

#     servo_test.min()
    
#     time.sleep(1)

#     servo_test.mid()
#     time.sleep(1)

#     servo_test.max()
#     time.sleep(1)
# finally:

#     pass

# # import sys
# # import os

# # sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# # import time
# # from hardware.servo_driver import AngleServoPWM  # import the class from the other file

# # # initialize the servo
# # servo = AngleServoPWM(pwm_channel=2, chip=0)  # adjust channel/chip if needed

# # # Initialize the servo

import RPi.GPIO as GPIO
import time

SERVO_PIN = 18  # GPIO pin connected to the servo

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM frequency
pwm.start(0)

def hold_angle(angle):
    # Clamp angle 0-180 just in case
    angle = max(0, min(180, angle))
    duty = 2.5 + (angle / 180.0) * 10  # SG90 duty cycle mapping
    pwm.ChangeDutyCycle(duty)
    # Keep the PWM running — do not turn off
    print(f"Holding servo at {angle}° with duty cycle {duty}%")

try:
    pwm.ChangeDutyCycle(5.0)
    # Example: hold 90 degrees
    # hold_angle(90)
    # while True:
    time.sleep(5)  # keep the script running so the servo holds position

    hold_angle(180)
    # while True:
    time.sleep(5)  # keep the script running so the servo holds position


except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
    print("Servo stopped and GPIO cleaned up")