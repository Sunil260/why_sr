import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from drivebase import DriveBase

db = DriveBase()

print("Forward")

db.set_Velocity(0.5, 0)

time.sleep(0.5)

print("Stop")

db.stop()