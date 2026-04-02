import cv2
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception

import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception
from controllers import AlignmentController, ApproachController
from controllers import LineFollowingController
from drivebase import DriveBase
import numpy as np
from enum import Enum

# Controller gains
BASE_SPEED = 0.3
LOOKAHEAD = 50

cam = OpenCVCamera()
p = Perception(cam)
# LINE FOLLOW → DETECT GREEN → ALIGN → DROP → DONE

class RobotState(Enum):
    LINE_FOLLOW = 1
    ALIGN_GREEN = 2
    DROP = 3
    DONE = 4

def run_line_follow(p, frame, dt, drivebase, line_follower, lookahead, base_speed):
    
    red_line_data = p.detect_red_line(frame, lookahead)
    p.show_line_debug(frame, red_line_data, lookahead)

    if not red_line_data.detected:
        print("Line lost")
        drivebase.stop()
        time.sleep(0.05)
        return   

    command = line_follower.compute(red_line_data, dt, base_speed)
    print(f"lin_v = {command.v:.2f} omega = {command.omega:.2f}")
    drivebase.set_Velocity(command.v, command.omega)

def run_align_green(p, frame, dt, drivebase, align_controller, aligned):
    """
    Align robot with green target (box or line)
    """

    result = p.analyze_green(frame)   # you will implement this

    if result.detected:
        print(
            f"Aligning to green: "
            f"e_x = {np.round(result.error_x, 2)} "
            f"e_y = {np.round(result.error_y, 2)} "
            f"area = {np.round(result.area, 2)}"
        )

        # ✅ alignment condition (same idea as blue)
        if abs(result.error_x) < align_controller.x_tol:
            aligned = True
            print("Aligned with green!")

        # control action
        command = align_controller.compute(result, dt)
        print(f"v = {command.v:.2f}, omega = {command.omega:.2f}")
        drivebase.set_Velocity(command.v, command.omega)

    else:
        print("Green not detected → stopping")
        drivebase.stop()

    return aligned

def main():
    cam = OpenCVCamera()
    p = Perception(cam,True)
    lw_detected = False
    aligned = False
    green_aligner = AlignmentController()
    approach_targer = ApproachController()
    drivebase = DriveBase()
    # From red_line_follow.py setting same controller values
    line_follower = LineFollowingController(k_heading_slow=0, v_min=0.25, v_max=0.7, omega_max=0.15)
    line_follower.lateral_pd.update_params(kp=0.2, kd=0.02)

    state = RobotState.LINE_FOLLOW #set state
    prev_t = time.monotonic()
    aligned = False


    try:
    

        while True:
            frame = cam.get_frame()
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now
            green = p.detect_green_box(frame)

            match state:
                case RobotState.LINE_FOLLOW:

                    if green.detected:
                        print(f"Green box detected | area={green.area:.0f} error_x={green.error_x}")
                        drivebase.stop()
                        state = RobotState.ALIGN_GREEN
                        continue

                    run_line_follow(p,frame,dt,drivebase,line_follower,LOOKAHEAD,BASE_SPEED)

                case RobotState.ALIGN_GREEN:
                    print(f"In green alignment mode now")
                    aligned = run_align_green(
                        p,
                        frame,
                        dt,
                        drivebase,
                        green_aligner,   # reuse your alignment controller
                        aligned
                    )

                    if aligned:
                        print("green box aligned")
                        drivebase.stop()
                        state = RobotState.DROP
                
                case RobotState.DROP:

                    print(f"drop off mode")

                
    except KeyboardInterrupt:
        print("Test stopped by user")
    finally:
        cam.release()
        cv2.destroyAllWindows()
                    
        


            # Skip GUI if no monitor
            # cv2.imshow("Green Box Test", frame)
            # if cv2.waitKey(1) == ord("q"):
            #     break

    # except KeyboardInterrupt:
    #     print("Test stopped by user")

    # # finally:
    # #     cam.release()
    # #     cv2.destroyAllWindows()