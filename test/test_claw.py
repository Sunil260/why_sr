import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from manipulator import Claw

claw = Claw()

print("Closing claw")
claw.open()

time.sleep(2)

print("Opening claw")
claw.close()

time.sleep(2)