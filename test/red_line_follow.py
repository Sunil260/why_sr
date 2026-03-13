import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from drivebase import DriveBase
from perception import OpenCVCamera, Perception


# Controller gains
K_X = 0.003        # cross track gain
K_THETA = 1.2      # heading gain

BASE_SPEED = 0.35
LOOKAHEAD = 200


def main():

    cam = OpenCVCamera()
    p = Perception(cam)
    drive = DriveBase()

    print("Starting red line follow test")

    try:

        while True:

            frame = cam.get_frame()

            res = p.detect_red_line(frame, LOOKAHEAD)

            if not res.detected:
                print("Line lost")
                drive.stop()
                time.sleep(0.05)
                continue

            # Combine cross-track and heading error
            angular = K_X * res.x_error_center + K_THETA * res.heading_error_ahead

            drive.set_Velocity(BASE_SPEED, angular)

            print(
                f"x_center={res.x_error_center:.2f}  "
                f"x_ahead={res.x_error_ahead:.2f}  "
                f"heading={res.heading_error_ahead:.3f}  "
                f"angular={angular:.3f}"
            )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        drive.stop()
        cam.release()


if __name__ == "__main__":
    main()