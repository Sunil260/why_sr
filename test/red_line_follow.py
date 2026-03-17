import sys
import os
import time
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from drivebase import DriveBase
from perception import OpenCVCamera, Perception
from controllers import LineFollowingController




# Controller gains
BASE_SPEED = 0.35
LOOKAHEAD = 100


def main():

    cam = OpenCVCamera()
    p = Perception(cam, True)
    drive = DriveBase()
    line_follower = LineFollowingController( )

    print("Starting red line follow test")

    try:

        prev_t = time.monotonic()

        while True:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frame = cam.get_frame()

            red_line_data = p.detect_red_line(frame, LOOKAHEAD)
            p.show_line_debug(frame, red_line_data,LOOKAHEAD)

            if not red_line_data.detected:
                print("Line lost")
                drive.stop()
                time.sleep(0.05)
                continue

            command = line_follower.compute(red_line_data, dt, BASE_SPEED)
            
            drive.set_Velocity(command.v, command.omega)

            print(
                f"x_center={red_line_data.x_error_center:.2f}  "
                f"x_ahead={red_line_data.x_error_ahead:.2f}  "
                f"heading={np.degrees(red_line_data.heading_error_ahead):.3f}  "
            )

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        drive.stop()
        cam.release()
        p.close_debug()


if __name__ == "__main__":
    main()