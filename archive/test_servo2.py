from gpiozero import Servo
import time

# servo = Servo(18)

# try:
#     # print("mid")
#     # servo.mid()
#     # time.sleep(5)
#     # print("max")
#     # servo.max()
#     # time.sleep(2)
#     # print("return")
#     # servo.min()
#     # time.sleep(2)
#     # servo.detach()

#     while(True):
#         angle = input("servo angle").strip().lower()
#         time.sleep(4)
#         print("set")
#         servo.value = (float(angle)/90)
#         time.sleep(1)

# except KeyboardInterrupt:
#     servo.detach()
import cv2
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import time
from manipulator import Claw

claw = Claw()

try:
    for angle in [-90, 20]:
        print(f"Angle: {angle}")
        claw.set_angle(angle)
        time.sleep(1.5)

finally:
    claw.end_servo()