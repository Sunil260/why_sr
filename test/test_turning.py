import time
from drivebase import DriveBase

db = DriveBase()

print("Spin in place")

db.set_Velocity(0, 0.4)

time.sleep(3)

db.stop()