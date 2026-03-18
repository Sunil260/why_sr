import time
import cv2
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception
from drivebase import DriveBase
from manipulator import Claw
from controllers import ApproachController
import numpy as np


def main():

    cam = OpenCVCamera()
    p = Perception(cam,True)
    approach_target = ApproachController()
    drivebase = DriveBase()
    claw = Claw()

    try:
        prev_t = time.monotonic()

        while True:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frame = cam.get_frame()
        
            #approach control
            result = p.detect_legoman(frame)
            if result.detected:
                print(
                    f"{result.detected} "
                    f"e_x = {np.round(result.e_x,2 )} "
                    f"e_y = {np.round(result.e_y,2)} "
                    f"head area =  {np.round(result.area,2)} "
                )
                cmd = approach_target.compute(result, dt)
                drivebase.set_Velocity(cmd.v,cmd.omega)
                if (result.centroid_y>=approach_target.pickup_y):
                    print("closing")
                    claw.close()
            else:
                print("no lego man")
    
    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        drivebase.stop()
        cam.release()
        p.close_debug()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    claw = Claw()
    claw.
    main()