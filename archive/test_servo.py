# # from gpiozero import Servo
# import time
# import sys
# import os
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
# from manipulator import Claw
# from hardware.servo_driver import ContinuousServoPWM


# servo = ContinuousServoPWM(pwm_channel=2, chip=0, center_us=1570, hz=50, min_us=1000, max_us=2000)

# try:
#     servo.set_pulse_us(1000)
#     time.sleep(5)
# except KeyboardInterrupt:
#     print("Stopping")

# #     print("Finding zero pulse width (servo should stop)...")
    
# #     # Start with center
# #     pulse = servo.center_us
# #     step = 5  # microseconds to adjust per step
    
# #     # We'll adjust slightly up and down to find the dead zone
# #     found_zero = False
# #     while not found_zero:
# #         # Try current pulse
# #         servo.set_pulse_us(pulse)
# #         print(f"Testing pulse: {pulse} µs")
# #         time.sleep(0.5)  # give servo time to react
        
# #         # Ask the user to check: is servo moving? (manual observation)
# #         response = input("Is the servo moving? (y/n) ").strip().lower()
# #         if response == 'n':
# #             found_zero = True
# #             print(f"Servo stopped at {pulse} µs")
# #             break
        
# #         # If it moves clockwise, increase pulse
# #         # If it moves counterclockwise, decrease pulse
# #         # Here we just increment to let user find direction
# #         direction = input("Move CW or CCW? (cw/ccw) ").strip().lower()
# #         if direction == 'cw':
# #             pulse += step
# #         elif direction == 'ccw':
# #             pulse -= step
# #         else:
# #             print("Invalid input, exiting.")
# #             break

# #     # Save the zero pulse for later
# #     zero_pulse = pulse
# #     print(f"Zero pulse width determined: {zero_pulse} µs")
# #     servo.stop()
    
# # finally:
# #     servo.close()

# # servo_test = Servo(18, min_pulse_width=0.5/1000, max_pulse_width =2.5/1000)

# # try:

# #     servo_test.min()
    
# #     time.sleep(1)

# #     servo_test.mid()
# #     time.sleep(1)

# #     servo_test.max()
# #     time.sleep(1)
# # finally:

# #     pass

import RPi.GPIO as GPIO
import time

# GPIO setup
SERVO_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Start PWM at 50 Hz
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz = 20 ms period
pwm.start(0)  # start with 0 duty cycle

try:
    # STOP the servo (dead zone pulse)
    stop_pulse_us = 1570  # adjust slightly if needed for your servo
    duty_cycle = (stop_pulse_us / 20000) * 100  # convert µs to %
    pwm.ChangeDutyCycle(duty_cycle)
    print(f"Holding servo still with {stop_pulse_us} µs pulse ({duty_cycle:.2f}% duty)")

    while True:
        time.sleep(1)  # keep running so PWM continues

except KeyboardInterrupt:
    print("Stopping PWM and cleaning up")

finally:
    pwm.stop()
    GPIO.cleanup()
