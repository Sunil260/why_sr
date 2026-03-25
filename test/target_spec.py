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
from collections import deque
from enum import Enum



# Controller gains
BASE_SPEED = 0.65
#.45
LOOKAHEAD = 75

omega_history = deque(maxlen=10)  # last 10 values
target_history = deque(maxlen=10)  # last 10 values
home = False

# trying to do a simple state machine between line follow mode --> target mode
class RobotState(Enum):
    LINE_FOLLOW = 1
    TARGET_MODE = 2
    LEGO_ALIGN = 3
    INTAKE = 4
    TURN = 5
    HOME = 6
    RECOVERY = 7

def run_line_follow(p, frame, dt, drivebase, line_follower, lookahead, base_speed):
    
    red_line_data = p.detect_red_line(frame, lookahead)
    p.show_line_debug(frame, red_line_data, lookahead)
    blue = p.detect_target_cheap(frame, min_area=2500)

    if not red_line_data.detected:
        print("Line lost")
        drivebase.stop()
        return RobotState.RECOVERY 

    command = line_follower.compute(red_line_data, dt, base_speed)
    
    omega_history.append(command.omega)

    print(f"lin_v = {command.v:.2f} omega = {command.omega:.2f}")
    drivebase.set_Velocity(command.v, command.omega)

    if blue.detected and not home:
        print(f"found w {blue.bpx} px" f"output.detected={blue.detected}")
        drivebase.stop(coast = False)
        print("BRAKEEEEE")

        return RobotState.TARGET_MODE
    
    return RobotState.LINE_FOLLOW

def run_target_mode(p, frame, dt, drivebase, target_aligner, approach_controller, aligned):

    if not aligned:
        # Step 1: Align to blue
        result = p.analyze_target(frame)
        target_history.append(result.error_x)  # store error_x for history
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
            #avg the error x for the result hist
            if len(target_history) > 0:
                result.error_x = np.mean(target_history)
            
            command = target_aligner.compute(result, dt)

            omega = command.omega
            print(f"omega: {omega}")
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
            return RobotState.TARGET_MODE

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
    # time.sleep(0.5)        # optional small delay to ensure claw closes
    print("Gdone close")
    for i in range(10):
        omega_history.append(1)
    home = True
    return RobotState.TURN      # return the next FSM state

def turn_until_line(p, frame, dt, drivebase, turn_controller):

    red_line_data = p.detect_red_line(frame, lookahead_y=100)

    drivebase.set_Velocity(0, -0.3)
    print(f"turning after target")

    if not red_line_data.detected: #i want this to be less than a certain area of red but ok for now...
        print(" red detected → stop turn")
        drivebase.stop(coast=False)
        return RobotState.HOME

    return RobotState.TURN

def run_recovery(p, frame, drivebase):

    red_line_data = p.detect_red_line(frame, lookahead_y=100)

    # If line found → go back
    if red_line_data.detected and not home:
        print("Line reacquired → back to line follow")
        drivebase.stop(coast=False)
        return RobotState.LINE_FOLLOW
    
    elif red_line_data.detected and  home:
        print("Line reacquired → back to line follow")
        drivebase.stop(coast=False)
        return RobotState.HOME

    # Turn opposite of last omega
    recovery_omega = get_recovery_direction() * 0.2

    # If last omega was ~0 (edge case), just pick a direction
    if abs(np.mean(omega_history)) < 0.01:
        recovery_omega = 0.2

    print(f"Recovering... omega = {recovery_omega:.2f}")

    drivebase.set_Velocity(0.0, recovery_omega)

    return RobotState.RECOVERY

def get_recovery_direction():
    if len(omega_history) == 0:
        return 1  # default direction

    # Option 1: average (smooth)
    avg = np.mean(omega_history)

    # # Option 2 (better): majority vote on sign
    # signs = np.sign(omega_history)
    # vote = np.sum(signs)

    # if abs(vote) > 0:
    #     return np.sign(vote)

    # fallback if tied/noisy
    return np.sign(avg) if abs(avg) > 0.01 else 1


def main():

    cam = OpenCVCamera()
    p = Perception(cam,False)
    lw_detected = False
    aligned = False
    target_aligner = AlignmentController(omega_max=0.2)
    approach_targer = ApproachController(omega_max=0.2,v_max=0.15,pickup_y=350,x_tol=0.08)
    drivebase = DriveBase()
    # From red_line_follow.py setting same controller values
    line_follower = LineFollowingController(k_heading_slow=5, v_min=0.25, v_max=0.7, omega_max=0.3)
    line_follower.lateral_pd.update_params(kp=0.2, kd=0.02)
    claw = Claw()
    turn_controller = TurnUntilLineController()


    state = RobotState.LINE_FOLLOW #set state
    # state = RobotState.TURN
   
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

                case RobotState.LEGO_ALIGN:
                    state = run_lego_align(p,frame,dt,drivebase,approach_targer)

                case RobotState.INTAKE:
                    state = grab_lego(claw)

                case RobotState.TURN:
                    # pass
                    state = turn_until_line(p,frame,dt,drivebase,turn_controller)

                case RobotState.HOME:
                    home = True
                    state =  run_line_follow(p,frame,dt,drivebase,line_follower,LOOKAHEAD,BASE_SPEED)

                case RobotState.RECOVERY:
                    state = run_recovery(p, frame, drivebase)




    except KeyboardInterrupt:
        print("Stopping robot")
        claw.open()


    finally:
        drivebase.stop()
        cam.release()
        p.close_debug()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()