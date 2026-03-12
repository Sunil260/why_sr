import time
import cv2

from perception import OpenCVCamera, Perception
from drivebase import DriveBase


# --- Robot Setup ---

cam = OpenCVCamera()
perception = Perception(cam)
drive = DriveBase()

DT = 0.05

# Tunable gains
LINE_KP = 0.004
GREEN_KP = 0.003

LINE_SPEED = 0.25
GREEN_SPEED = 0.2


print("Starting line following test")
print("Robot will switch to green box when detected")

try:

    while True:

        frame = cam.get_frame()

        # --- perception ---
        line = perception.detect_red_line(frame)
        green = perception.detect_green_box(frame)

        # --- behavior switch ---
        if green.detected:

            # drive toward green box
            error = green.error_x

            omega = -GREEN_KP * error

            drive.set_Velocity(GREEN_SPEED, omega)

            print(f"GREEN TARGET | error {error:.1f}")

        elif line.detected:

            # follow red line
            error = line.x_error_center

            omega = -LINE_KP * error

            drive.set_Velocity(LINE_SPEED, omega)

            print(f"FOLLOW LINE | error {error:.1f}")

        else:

            # lost everything
            drive.set_Velocity(0.0, 0.0)
            print("No features detected")

        cv2.imshow("Robot View", frame)

        if cv2.waitKey(1) == ord("q"):
            break

        time.sleep(DT)

except KeyboardInterrupt:
    pass

finally:

    print("Stopping robot")

    drive.stop()
    cam.release()
    cv2.destroyAllWindows()