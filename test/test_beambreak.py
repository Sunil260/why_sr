import time
from manipulator import Claw

claw = Claw()

print("Insert object to break beam")

while True:

    if claw.is_object_detected():
        print("Object detected!")

    time.sleep(0.1)