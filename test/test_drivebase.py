import time
from drivebase import DriveBase

db = DriveBase()

print("Forward")

db.set_Velocity(0.4, 0)

time.sleep(3)

print("Stop")

db.stop()