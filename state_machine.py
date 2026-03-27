import cv2
import sys
import os
import time
from collections import deque
from enum import Enum, auto

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception
from controllers import (
    AlignmentController,
    ApproachController,
    LineFollowingController,
)
from drivebase import DriveBase
from manipulator import Claw

BASE_SPEED = 0.3
LOOKAHEAD = 75

class RobotState(Enum):
    LINE_FOLLOW_OUTBOUND = auto()
    ALIGN_TARGET = auto()
    LEGO_ALIGN = auto()
    INTAKE = auto()
    TURN_TO_LINE = auto()
    LINE_FOLLOW_HOME = auto()
    RECOVERY = auto()
    STOP = auto()

class RobotFSM:
    def __init__(self, p, drivebase, claw, line_follower, target_aligner, approach_controller):
        self.p = p
        self.drivebase = drivebase
        self.claw = claw

        self.line_follower = line_follower
        self.target_aligner = target_aligner
        self.approach_controller = approach_controller

        self.state = RobotState.LINE_FOLLOW_OUTBOUND
        self.has_object = False

        self.omega_history = deque(maxlen=10)
        self.target_history = deque(maxlen=10)

        self.intake_start = None
        self.intake_duration = 0.6

    def transition(self, new_state):
        if new_state != self.state:
            print(f"[FSM] {self.state.name} -> {new_state.name}")
        self.state = new_state

        if new_state != RobotState.ALIGN_TARGET:
            self.target_history.clear()

        if new_state != RobotState.INTAKE:
            self.intake_start = None

    def get_recovery_direction(self):
        if not self.omega_history:
            return 1.0
        avg = np.mean(self.omega_history)
        return np.sign(avg) if abs(avg) > 0.01 else 1.0

    def run_line_follow(self, frame, dt):
        red = self.p.detect_red_line(frame, LOOKAHEAD)
        self.p.show_line_debug(frame, red, LOOKAHEAD)

        if not red.detected:
            print("Line lost")
            self.drivebase.stop()
            return RobotState.RECOVERY
        
        blue_t = self.p.detect_target_cheap(frame, min_area=20)

        #
        if not self.has_object:
            blue = self.p.detect_target_cheap(frame, min_area=500)
            if blue.detected:
                print(f"Target candidate found: {blue.bpx}px")
                self.omega_history.clear()
                self.drivebase.stop(coast=False)
                return RobotState.LEGO_ALIGN
    
        cmd = self.line_follower.compute(red, dt, BASE_SPEED)
        if hasattr(blue_t, "bpx") and blue_t.bpx > 600:
            cmd.v *= 0.5
            cmd.omega *= 0.25

        print(f"Vel: {cmd.v} Omega: {cmd.omega}")
        self.omega_history.append(cmd.omega)
        self.drivebase.set_Velocity(cmd.v, cmd.omega)

        return self.state

    def run_align_target(self, frame, dt):
        result = self.p.analyze_target(frame)

        if not result.detected:
            print("Blue target not detected")
            self.drivebase.stop()
            return RobotState.ALIGN_TARGET

        self.target_history.append(result.error_x)
        result.error_x = float(np.mean(self.target_history))

        if abs(result.error_x) < self.target_aligner.x_tol:
            self.drivebase.stop(coast=False)
            return RobotState.LEGO_ALIGN

        cmd = self.target_aligner.compute(result, dt)
        omega = cmd.omega
        if abs(omega) > 0.01:
            omega = np.sign(omega) * max(abs(omega), 0.18)

        self.drivebase.set_Velocity(0.0, omega)
        return RobotState.ALIGN_TARGET

    def run_lego_align(self, frame, dt):
        result = self.p.detect_legoman(frame)

        if not result.detected:
            print("Lego not detected")
            self.drivebase.stop()
            return RobotState.LEGO_ALIGN

        print(f"Lego: error_x={result.e_x:.2f}, error_y={result.centroid_y:.2f}")

        if result.centroid_y >= self.approach_controller.pickup_y :
            self.drivebase.stop(coast=False)
            return RobotState.INTAKE

        cmd = self.approach_controller.compute(result, dt)
        omega = cmd.omega
        if abs(omega) > 0.01:
            omega = np.sign(omega) * max(abs(omega), 0.2)

        self.drivebase.set_Velocity(cmd.v, omega)
        return RobotState.LEGO_ALIGN

    def run_intake(self, now):
        if self.intake_start is None:
            self.intake_start = now
            print("Closing claw")
            self.claw.close()

        if now - self.intake_start >= self.intake_duration:
            self.has_object = True
            self.omega_history.clear()
            self.omega_history.extend([1.0] * 10)
            return RobotState.TURN_TO_LINE

        return RobotState.INTAKE

    def run_turn_to_line(self, frame):
        red = self.p.detect_red_line(frame, lookahead_y=100)

        if red.detected:
            print("Line found again")
            self.drivebase.stop(coast=False)
            return RobotState.LINE_FOLLOW_HOME

        self.drivebase.set_Velocity(0.0, 0.3)
        return RobotState.TURN_TO_LINE

    def run_recovery(self, frame):
        red = self.p.detect_red_line(frame, lookahead_y=LOOKAHEAD)

        if red.detected:
            self.drivebase.stop(coast=False)
            return RobotState.LINE_FOLLOW_HOME if self.has_object else RobotState.LINE_FOLLOW_OUTBOUND

        omega = self.get_recovery_direction() * 0.2
        self.drivebase.set_Velocity(0.0, omega)
        return RobotState.RECOVERY

    def step(self, frame, dt, now):
        if self.state == RobotState.LINE_FOLLOW_OUTBOUND:
            self.transition(self.run_line_follow(frame, dt))

        elif self.state == RobotState.ALIGN_TARGET:
            self.transition(self.run_align_target(frame, dt))

        elif self.state == RobotState.LEGO_ALIGN:
            self.transition(self.run_lego_align(frame, dt))

        elif self.state == RobotState.INTAKE:
            self.transition(self.run_intake(now))

        elif self.state == RobotState.TURN_TO_LINE:
            self.transition(self.run_turn_to_line(frame))

        elif self.state == RobotState.LINE_FOLLOW_HOME:
            self.transition(self.run_line_follow(frame, dt))

        elif self.state == RobotState.RECOVERY:
            self.transition(self.run_recovery(frame))

        elif self.state == RobotState.STOP:
            self.drivebase.stop(coast=False)

def main():
    cam = OpenCVCamera()
    p = Perception(cam, False)
    drivebase = DriveBase()
    claw = Claw()

    target_aligner = AlignmentController(omega_max=0.2, x_tol=0.02)
    target_aligner.align_pd.update_params(kp=0.05, kd=0.0)

    approach_controller = ApproachController(omega_max=0.175, v_max=0.15, pickup_y=350, x_tol=0.08)
    approach_controller.lateral_pd.update_params(kp=0.1, kd=0.02)

    line_follower = LineFollowingController(k_heading_slow=2, v_min=0.25, v_max=0.7, omega_max=0.15)
    line_follower.lateral_pd.update_params(kp=0.2, kd=0.04)

    fsm = RobotFSM(p, drivebase, claw, line_follower, target_aligner, approach_controller)

    prev_t = time.monotonic()
    claw.open()

    try:
        while True:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frame = cam.get_frame()
            fsm.step(frame, dt, now)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping robot")
        claw.open()
        time.sleep(1)

    finally:
        drivebase.stop()
        cam.release()
        p.close_debug()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()