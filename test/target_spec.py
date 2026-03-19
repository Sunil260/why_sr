import cv2
import sys
import os
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

# trying to do a simple state machine between line follow mode --> target mode
class RobotState(Enum):
    LINE_FOLLOW = 1
    TARGET_MODE = 2

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

def run_target_mode(p, frame, dt, drivebase, target_aligner, approach_controller, aligned):
    """
    Runs the target mode behavior:
    1. Align to blue target
    2. Approach the Lego target
    """

    if not aligned:
        # Step 1: Align to blue
        result = p.analyze_target(frame)
        if result.detected:
            print(
                f"Aligning: detected={result.detected} "
                f"e_x={np.round(result.error_x, 2)} "
                f"e_y={np.round(result.error_y, 2)} "
                f"Blue area={np.round(result.area, 2)}"
            )

            # Check if within tolerance
            if abs(result.error_x) < target_aligner.x_tol:
                aligned = True
                print("Aligned to blue target")

            # Compute alignment command
            command = target_aligner.compute(result, dt)
            print(f"Alignment command → v: {command.v:.2f}, omega: {command.omega:.2f}")
            drivebase.set_Velocity(command.v, command.omega)

        else:
            print("Blue target not detected, stopping.")
            drivebase.stop()

    else:
        # Step 2: Approach the Lego target
        result = p.detect_legoman(frame)
        if result.detected:
            print(
                f"Approaching: detected={result.detected} "
                f"e_x={np.round(result.e_x, 2)} "
                f"e_y={np.round(result.e_y, 2)} "
                f"Head area={np.round(result.area, 2)}"
            )
            # Compute approach command
            command = approach_controller.compute(result, dt)
            print(f"Approach command → v: {command.v:.2f}, omega: {command.omega:.2f}")
            drivebase.set_Velocity(command.v, command.omega)
        else:
            print("Lego target not detected, stopping.")
            drivebase.stop()

    return aligned



def main():

    cam = OpenCVCamera()
    p = Perception(cam,True)
    lw_detected = False
    aligned = False
    target_aligner = AlignmentController()
    approach_targer = ApproachController()
    drivebase = DriveBase()
    # From red_line_follow.py setting same controller values
    line_follower = LineFollowingController(k_heading_slow=0, v_min=0.25, v_max=0.7, omega_max=0.15)
    line_follower.lateral_pd.update_params(kp=0.2, kd=0.02)


    state = RobotState.LINE_FOLLOW #set state


    prev_t = time.monotonic()


    try:
        # prev_t = time.monotonic()

        while True:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frame = cam.get_frame()

# check for blue
            blue = p.detect_target_cheap(frame)

            # match-case for FSM
            match state:
                case RobotState.LINE_FOLLOW:

                    # transistion to blue if blue detected
                    if blue.detected:
                        print(f"found w {blue.bpx} px" f"output.detected={blue.detected}")
                        drivebase.stop()
                        state = RobotState.TARGET_MODE
                        continue
                    
                    # red line follow here
                    run_line_follow(p,frame,dt,drivebase,line_follower,LOOKAHEAD,BASE_SPEED)
                    # print("Starting red line follow")
                    # red_line_data = p.detect_red_line(frame, 50)  # LOOKAHEAD
                    # p.show_line_debug(frame, red_line_data, 50)

                    # if not red_line_data.detected:
                    #     print("Line lost")
                    #     drivebase.stop()
                    #     time.sleep(0.05)
                    #     continue

                    # command = line_follower.compute(red_line_data, dt, 0.3)  # BASE_SPEED
                    # print(f"lin_v = {command.v:.2f} omega = {command.omega:.2f}")
                    # drivebase.set_Velocity(command.v, command.omega)


                case RobotState.TARGET_MODE:
                    print(f"In target state now")
                    print(f"found w {blue.bpx} px" f"output.detected={blue.detected}")
                    # aligned = run_target_mode(p,frame,dt,drivebase,target_aligner,approach_targer,aligned)





    except KeyboardInterrupt:
        print("Stopping robot")

            

    finally:
        drivebase.stop()
        cam.release()
        p.close_debug()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()