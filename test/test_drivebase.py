import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from drivebase import DriveBase

db = DriveBase()

print("Forward")

db.set_Velocity(0.4, 0)

time.sleep(3)

print("Stop")

db.stop()