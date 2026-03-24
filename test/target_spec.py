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
from manipulator import Claw
from controllers import TurnUntilLineController
from enum import Enum

# Controller gains
BASE_SPEED = 0.2
LOOKAHEAD = 50

# trying to do a simple state machine between line follow mode --> target mode
class RobotState(Enum):
    LINE_FOLLOW = 1
    TARGET_MODE = 2
    LEGO_ALIGN = 3
    INTAKE = 4
    TURN = 5

def run_line_follow(p, frame, dt, drivebase, line_follower, lookahead, base_speed):
    
    red_line_data = p.detect_red_line(frame, lookahead)
    p.show_line_debug(frame, red_line_data, lookahead)
    blue = p.detect_target_cheap(frame, min_area=1500)

    if not red_line_data.detected:
        print("Line lost")
        drivebase.stop()
        return RobotState.LINE_FOLLOW  

    command = line_follower.compute(red_line_data, dt, base_speed)
    print(f"lin_v = {command.v:.2f} omega = {command.omega:.2f}")
    drivebase.set_Velocity(command.v, command.omega)

    if blue.detected:
        print(f"found w {blue.bpx} px" f"output.detected={blue.detected}")
        drivebase.stop()
        time.sleep(1)
        return RobotState.TARGET_MODE
    
    return RobotState.LINE_FOLLOW

def run_target_mode(p, frame, dt, drivebase, target_aligner, approach_controller, aligned):

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
                return RobotState.LEGO_ALIGN

            # Compute alignment command
            command = target_aligner.compute(result, dt)

            omega = command.omega
            # v = command.velocity

            # Deadzone compensation
            if abs(result.error_x) < target_aligner.x_tol:
                omega = 0.0
                aligned = True
            else:
                omega = command.omega

                # apply minimum turn speed ONLY if turning is needed
                if abs(omega) > 0.01:
                    omega = np.sign(omega) * max(abs(omega), 0.2)

            drivebase.set_Velocity(0, omega)   

        else:
            print("Blue target not detected, stopping.")
            drivebase.stop()
            return RobotState.LEGO_ALIGN

    return RobotState.TARGET_MODE

def run_lego_align(p, frame, dt, drivebase, approach_controller):

    result = p.detect_legoman(frame)

    if not result.detected:
        print("Lego not detected")
        drivebase.stop()
        return RobotState.LEGO_ALIGN

    print(
        f"Lego: e_x={np.round(result.e_x,2)} "
        f"y={np.round(result.centroid_y,2)}"
    )

    if result.centroid_y >= approach_controller.pickup_y:
        print("Lego reached pickup line → stopping")  # debug
        drivebase.stop()  # full stop
        return RobotState.INTAKE  # could transition to next state if desired

    command = approach_controller.compute(result, dt)

    # deadzone compensation for turning
    omega = command.omega
    if abs(omega) > 0.01:
        omega = np.sign(omega) * max(abs(omega), 0.2)

    drivebase.set_Velocity(command.v, omega)

    # stop condition (same as controller)
    if command.v == 0.0 and command.omega == 0.0:
        print("Reached Lego")
        drivebase.stop()
        return RobotState.INTAKE  # or DONE if you add it later

    return RobotState.LEGO_ALIGN

def grab_lego(claw):

    print("Grabbing Lego → closing claw")
    claw.close()           # close the claw
    time.sleep(0.5)        # optional small delay to ensure claw closes
    return RobotState.TURN      # return the next FSM state

def turn_until_line(p, frame, dt, drivebase, turn_controller):

    # Detect red line in current frame
    red_line_data = p.detect_red_line(frame, lookahead=100)  # adjust lookahead if needed

    # Compute drive command from controller
    command = turn_controller.compute(red_line_data, dt)

    # Apply command to robot
    drivebase.set_Velocity(command.v, command.omega)

    # Check if line detected → transition to next state
    if red_line_data.detected:
        print("Red line detected → stopping turn")
        drivebase.stop()
        return RobotState.LINE_FOLLOW

    # Keep spinning if not detected
    return RobotState.TURN  # stay in turn state

def main():

    cam = OpenCVCamera()
    p = Perception(cam,False)
    lw_detected = False
    aligned = False
    target_aligner = AlignmentController()
    approach_targer = ApproachController(omega_max=0.2,v_max=0.2,pickup_y=400,x_tol=0.05)
    drivebase = DriveBase()
    # From red_line_follow.py setting same controller values
    line_follower = LineFollowingController(k_heading_slow=0, v_min=0.25, v_max=0.7, omega_max=0.15)
    line_follower.lateral_pd.update_params(kp=0.2, kd=0.02)
    claw = Claw()
    turn_controller = TurnUntilLineController()

    state = RobotState.LINE_FOLLOW #set state

    prev_t = time.monotonic()
    claw.open()


    try:
        # prev_t = time.monotonic()

        while True:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frame = cam.get_frame()            

            # match-case for FSM
            match state:
                case RobotState.LINE_FOLLOW:                
                    
                    # red line follow here
                    state = run_line_follow(p,frame,dt,drivebase,line_follower,LOOKAHEAD,BASE_SPEED)

                case RobotState.TARGET_MODE:
                    # print(f"found w {blue.bpx} px" f"output.detected={blue.detected}")
                    # drivebase.stop()
                    state = run_target_mode(p,frame,dt,drivebase,target_aligner,approach_targer,aligned)
                    if aligned:
                        print("aligned")    

                case RobotState.LEGO_ALIGN:
                    state = run_lego_align(p,frame,dt,drivebase,approach_targer)

                case RobotState.INTAKE:
                    state = grab_lego(claw)

                case RobotState.TURN:
                    state = run_until_line(p,frame,dt,drivebase,turn_controller)




    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        drivebase.stop()
        cam.release()
        p.close_debug()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()