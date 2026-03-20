import cv2
import sys
import os
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception
from controllers import AlignmentController, ApproachController
from controllers import LineFollowingController
from drivebase import DriveBase
from manipulator import Claw
from controllers import TurnUntilLineController
import numpy as np
from enum import Enum

# Controller gains
BASE_SPEED = 0.3
LOOKAHEAD = 50
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

            omega = command.omega

            # Deadzone compensation
            if abs(omega) > 0.01:
                omega = np.sign(omega) * max(abs(omega), 0.17)

            drivebase.set_Velocity(0.0, omega)   

        else:
            print("Blue target not detected, stopping.")
            drivebase.stop()

    return aligned

def approach_lego_target(p, frame, dt, drivebase, approach_controller):
    """
    Detects and approaches the Lego target.
    
    Returns:
        detected (bool): True if Lego target detected.
    """
    result = p.detect_legoman(frame)
    detected = False

    if result.detected:
        detected = True
        print(
            f"Approaching: detected={result.detected} "
            f"e_x={np.round(result.e_x, 2)} "
            f"e_y={np.round(result.e_y, 2)} "
            f"Head area={np.round(result.area, 2)}"
        )

        # Compute approach command
        command = approach_controller.compute(result, dt)
        print(f"Approach command → v: {command.v:.2f}, omega: {command.omega:.2f}")

        omega = command.omega
        # Deadzone compensation 
        if abs(omega) > 0.01:
            omega = np.sign(omega) * max(abs(omega), 0.2)
        drivebase.set_Velocity(command.v, omega)
    else:
        print("Lego target not detected, stopping.")
        drivebase.stop()

    return detected

# trying to do a simple state machine between line follow mode --> target mode
class RobotState(Enum):
    LINE_FOLLOW = 1 #follow the red line until blue seen
    TARGET_MODE = 2 #align to target until below x tolerance
    LEGO_ALIGN = 3 #detect lego and align claw until the lego is deep enough into the claw (400px)
    INTAKE_MODE = 4 #close the claw
    TURN = 5 #turn until red line seen
    LINE_FOLLOW = 5 #follow red line until green seen
    GREEN_ALIGN = 6 #align to green box
    OPEN_CLAW = 7 #release lego in box


# Initial state
current_state = RobotState.LINE_FOLLOW

def main():
    cam = OpenCVCamera()
    p = Perception(cam,True)
    lw_detected = False
    aligned = False
    target_aligner = AlignmentController()
    green_box_aligner = AlignmentController()
    approach_targer = ApproachController(omega_max=0.2,v_max=0.2,pickup_y=400,x_tol=0.05)
    turn_controller = TurnUntilController()
    drivebase = DriveBase()
    # From red_line_follow.py setting same controller values
    line_follower = LineFollowingController(k_heading_slow=0, v_min=0.25, v_max=0.7, omega_max=0.15)
    line_follower.lateral_pd.update_params(kp=0.2, kd=0.02)


    state = RobotState.LINE_FOLLOW #set state


    prev_t = time.monotonic()

    try:

        while True:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frame = cam.get_frame()

            blue = p.detect_target_cheap(frame, min_area=5000)
            green = p.detect_green_box(frame,min_area=100)

            match current_state:
                case RobotState.LINE_FOLLOW:

                    if blue.detected:
                        print(f"found w {blue.bpx} px" f"output.detected={blue.detected}")
                        drivebase.stop()
                        time.sleep(1)
                        state = RobotState.TARGET_MODE
                        continue
                    run_line_follow(p,frame,dt,drivebase,line_follower,LOOKAHEAD,BASE_SPEED)

                case RobotState.TARGET_MODE:
                    drivebase.stop()
                    print(f"In target state now")
                    print(f"found w {blue.bpx} px" f"output.detected={blue.detected}")
                    aligned = run_target_mode(p,frame,dt,drivebase,target_aligner,approach_targer,aligned)

                    if aligned:
                        current_state = RobotState.LEGO_ALIGN

                case RobotState.LEGO_ALIGN:

                    lego_detected = approach_lego_target(p, frame, dt, drivebase, approach_targer)
                    if lego_detected:
                        current_state = RobotState.INTAKE_MODE


                case RobotState.INTAKE_MODE:
                    print("Opening claw")
                    Claw.close()

                    time.sleep(2) # Transition criteria
                    
                    current_state = RobotState.TURN

                case RobotState.TURN:
                    estimate = p.detect_red_line(frame)  

                    # Compute command using the controller
                    command = turn_controller.compute(estimate, dt)

                    # Apply the command to the robot
                    drivebase.set_Velocity(command.v, command.omega)

                    # Transition: if line detected, go to next state
                    if estimate.detected:
                        print("Red line detected, stopping turn.")
                        current_state = RobotState.RED_LINE_FOLLOW

                case RobotState.LINE_FOLLOW:
                    if green.detected:
                        drivebase.stop()
                        current_state = RobotState.GREEN_ALIGN
                    
                    run_line_follow(p,frame,dt,drivebase,line_follower,LOOKAHEAD,BASE_SPEED)

                case RobotState.GREEN_ALIGN:
                    green_box_aligner
                    current_state = RobotState.OPEN_CLAW

                case RobotState.OPEN_CLAW:
                    drivebase.stop()
                    Claw.open()
                    time.sleep(2)
  

    except KeyboardInterrupt:
        print("Stopping robot")

            

    finally:
        drivebase.stop()
        cam.release()
        p.close_debug()
        cv2.destroyAllWindows()



print("Spec done")