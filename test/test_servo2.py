from gpiozero import Servo
import time

servo = Servo(18)

try:
    # print("mid")
    # servo.mid()
    # time.sleep(5)
    # print("max")
    # servo.max()
    # time.sleep(2)
    # print("return")
    # servo.min()
    # time.sleep(2)
    # servo.detach()

    while(True):
        angle = input("servo angle").strip().lower()
        time.sleep(4)
        print("set")
        servo.value = (float(angle)/90)
        time.sleep(1)

except KeyboardInterrupt:
    servo.detach()