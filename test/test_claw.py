import time
from manipulator import Claw

claw = Claw()

print("Opening claw")
claw.open()

time.sleep(2)

print("Closing claw")
claw.close()

time.sleep(2)