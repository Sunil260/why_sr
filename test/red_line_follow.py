import cv2
import sys
import os
import time
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from drivebase import DriveBase
from perception import OpenCVCamera, Perception
from controllers import LineFollowingController




# Controller gains
BASE_SPEED = 0.3
LOOKAHEAD = 100


def main():

    cam = OpenCVCamera()
    p = Perception(cam, False)
    drive = DriveBase()
    line_follower = LineFollowingController(k_heading_slow=1, v_min=0.25, v_max=0.7, omega_max=0.15)

    line_follower.lateral_pd.update_params(kp=0.2, kd=0.02)

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
            print(f"lin_v = {command.v:.2f} " f"omega = {command.omega:.2f}")
            drive.set_Velocity(command.v, command.omega)

            print(
                f"x_center={red_line_data.x_error_center:.2f}  "
                f"x_ahead={red_line_data.x_error_ahead:.2f}  "
                f"heading={np.degrees(red_line_data.heading_error_ahead):.3f}  "
            )

    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        drive.stop()
        cam.release()
        p.close_debug()


if __name__ == "__main__":
    main()