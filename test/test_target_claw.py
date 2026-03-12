import time
import cv2
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception
from drivebase import DriveBase
from manipulator import Claw

# --- Robot Setup ---
cam = OpenCVCamera()
perception = Perception(cam)
drive = DriveBase()
claw = Claw()

DT = 0.05

# Tunable gains
LINE_KP = 0.004
LINE_SPEED = 0.25

TARGET_APPROACH_SPEED = 0.2
TARGET_ALIGN_KP = 0.003

# Target area threshold to decide "close enough"
TARGET_AREA_THRESHOLD = 5000  # tune based on your box size

picked_up = False

print("Starting line following with pickup test")
print("Robot will follow line and close claw on green box detection")

try:

    while True:
        frame = cam.get_frame()

        # --- perception ---
        line = perception.detect_red_line(frame)
        green = perception.detect_green_box(frame)

        # --- behavior ---
        if picked_up:
            drive.set_Velocity(0.0, 0.0)
            print("Object picked up, robot stopped")
            break  # test complete

        elif green.detected:

            # Compute error in x
            error = green.error_x
            omega = -TARGET_ALIGN_KP * error

            # Reduce speed as it approaches
            speed = TARGET_APPROACH_SPEED
            if green.area >= TARGET_AREA_THRESHOLD:
                speed = 0.0
                print("Target reached, closing claw")
                claw.close()
                picked_up = True

            drive.set_Velocity(speed, omega)
            print(f"APPROACH GREEN | error {error:.1f}, area {green.area:.1f}")

        elif line.detected:

            # Follow red line
            error = line.x_error_center
            omega = -LINE_KP * error

            drive.set_Velocity(LINE_SPEED, omega)
            print(f"FOLLOW LINE | error {error:.1f}")

        else:

            drive.set_Velocity(0.0, 0.0)
            print("No features detected")

        cv2.imshow("Robot View", frame)
        if cv2.waitKey(1) == ord("q"):
            break

        time.sleep(DT)

except KeyboardInterrupt:
    pass

finally:
    print("Stopping robot and releasing camera")
    drive.stop()
    cam.release()
    cv2.destroyAllWindows()