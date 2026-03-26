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


BASE_SPEED = 0.45
LOOKAHEAD = 75


class RobotState(Enum):
    LINE_FOLLOW_OUTBOUND = auto()
    BRAKE_FOR_TARGET = auto()
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
        self.target_history = deque(maxlen=8)

        self.intake_start = None
        self.intake_duration = 0.6

        self.brake_start = None
        self.brake_duration = 0.18

        self.blue_seen_count = 0
        self.blue_seen_required = 2

        self.target_lost_count = 0
        self.target_lost_limit = 10

        self.lego_lost_count = 0
        self.lego_lost_limit = 10

        self.turn_search_omega = -0.3
        self.recovery_omega = 0.2

    def transition(self, new_state):
        if new_state != self.state:
            print(f"[FSM] {self.state.name} -> {new_state.name}")

        self.state = new_state

        if new_state != RobotState.ALIGN_TARGET:
            self.target_history.clear()
            self.target_lost_count = 0

        if new_state != RobotState.LEGO_ALIGN:
            self.lego_lost_count = 0

        if new_state != RobotState.INTAKE:
            self.intake_start = None

        if new_state != RobotState.BRAKE_FOR_TARGET:
            self.brake_start = None

        if new_state not in (RobotState.LINE_FOLLOW_OUTBOUND, RobotState.BRAKE_FOR_TARGET):
            self.blue_seen_count = 0

        self.reset_controllers()

    def reset_controllers(self):
        if hasattr(self.line_follower, "reset"):
            self.line_follower.reset()
        elif hasattr(self.line_follower, "lateral_pd"):
            self.line_follower.lateral_pd.reset()

        if hasattr(self.target_aligner, "reset"):
            self.target_aligner.reset()
        elif hasattr(self.target_aligner, "align_pd"):
            self.target_aligner.align_pd.reset()

        if hasattr(self.approach_controller, "reset"):
            self.approach_controller.reset()
        elif hasattr(self.approach_controller, "lateral_pd"):
            self.approach_controller.lateral_pd.reset()
        elif hasattr(self.approach_controller, "heading_pd"):
            self.approach_controller.heading_pd.reset()

    def get_recovery_direction(self):
        if not self.omega_history:
            return 1.0

        avg = float(np.mean(self.omega_history))
        return np.sign(avg) if abs(avg) > 0.01 else 1.0

    def run_line_follow(self, frame, dt, search_target: bool):
        red = self.p.detect_red_line(frame, LOOKAHEAD)
        self.p.show_line_debug(frame, red, LOOKAHEAD)

        if not red.detected:
            print("[LINE] line lost")
            self.drivebase.stop()
            return RobotState.RECOVERY

        cmd = self.line_follower.compute(red, dt, BASE_SPEED)

        if search_target and not self.has_object:
            blue = self.p.detect_target_cheap(frame, min_area=300)

            # gentle pre-slow when target is getting large
            if blue.detected and hasattr(blue, "bpx") and blue.bpx > 600:
                cmd.v *= 0.5
                cmd.omega *= 0.25

            # latch target over 2 frames
            if blue.detected:
                self.blue_seen_count += 1
            else:
                self.blue_seen_count = 0

            if self.blue_seen_count >= self.blue_seen_required:
                print(f"[LINE] target latched: bpx={getattr(blue, 'bpx', -1)}")
                self.drivebase.stop(coast=False)
                self.omega_history.clear()
                return RobotState.BRAKE_FOR_TARGET

        self.omega_history.append(cmd.omega)
        print(f"[LINE] v={cmd.v:.2f} omega={cmd.omega:.2f}")
        self.drivebase.set_Velocity(cmd.v, cmd.omega)
        return self.state

    def run_brake_for_target(self, now):
        self.drivebase.stop(coast=False)

        if self.brake_start is None:
            self.brake_start = now

        if now - self.brake_start >= self.brake_duration:
            return RobotState.ALIGN_TARGET

        return RobotState.BRAKE_FOR_TARGET

    def run_align_target(self, frame, dt):
        result = self.p.analyze_target(frame)

        if not result.detected:
            self.target_lost_count += 1
            print(f"[ALIGN_TARGET] target lost ({self.target_lost_count})")
            self.drivebase.stop()
            if self.target_lost_count > self.target_lost_limit:
                return RobotState.RECOVERY
            return RobotState.ALIGN_TARGET

        self.target_lost_count = 0

        self.target_history.append(result.error_x)
        result.error_x = float(np.mean(self.target_history))

        print(
            f"[ALIGN_TARGET] ex={result.error_x:.3f} "
            f"ey={result.error_y:.3f} area={result.area:.1f}"
        )

        if abs(result.error_x) < self.target_aligner.x_tol:
            self.drivebase.stop(coast=False)
            return RobotState.LEGO_ALIGN

        cmd = self.target_aligner.compute(result, dt)

        # no forced minimum omega here
        omega = max(-self.target_aligner.omega_max, min(self.target_aligner.omega_max, cmd.omega))
        self.drivebase.set_Velocity(0.0, omega)
        return RobotState.ALIGN_TARGET

    def run_lego_align(self, frame, dt):
        result = self.p.detect_legoman(frame)

        if not result.detected:
            self.lego_lost_count += 1
            print(f"[LEGO_ALIGN] lego lost ({self.lego_lost_count})")
            self.drivebase.stop()
            if self.lego_lost_count > self.lego_lost_limit:
                return RobotState.ALIGN_TARGET
            return RobotState.LEGO_ALIGN

        self.lego_lost_count = 0

        print(f"[LEGO_ALIGN] ex={result.e_x:.3f} y={result.centroid_y:.1f}")

        cmd = self.approach_controller.compute(result, dt)

        if cmd.v == 0.0 and cmd.omega == 0.0:
            self.drivebase.stop(coast=False)
            return RobotState.INTAKE

        # no forced minimum omega here either
        omega = max(-self.approach_controller.omega_max, min(self.approach_controller.omega_max, cmd.omega))
        self.drivebase.set_Velocity(cmd.v, omega)
        return RobotState.LEGO_ALIGN

    def run_intake(self, now):
        if self.intake_start is None:
            self.intake_start = now
            print("[INTAKE] closing claw")
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
            print("[TURN_TO_LINE] line found")
            self.drivebase.stop(coast=False)
            return RobotState.LINE_FOLLOW_HOME

        self.drivebase.set_Velocity(0.0, self.turn_search_omega)
        return RobotState.TURN_TO_LINE

    def run_recovery(self, frame):
        red = self.p.detect_red_line(frame, lookahead_y=100)

        if red.detected:
            self.drivebase.stop(coast=False)
            return RobotState.LINE_FOLLOW_HOME if self.has_object else RobotState.LINE_FOLLOW_OUTBOUND

        omega = self.get_recovery_direction() * self.recovery_omega
        print(f"[RECOVERY] omega={omega:.2f}")
        self.drivebase.set_Velocity(0.0, omega)
        return RobotState.RECOVERY

    def step(self, frame, dt, now):
        if self.state == RobotState.LINE_FOLLOW_OUTBOUND:
            self.transition(self.run_line_follow(frame, dt, search_target=True))

        elif self.state == RobotState.BRAKE_FOR_TARGET:
            self.transition(self.run_brake_for_target(now))

        elif self.state == RobotState.ALIGN_TARGET:
            self.transition(self.run_align_target(frame, dt))

        elif self.state == RobotState.LEGO_ALIGN:
            self.transition(self.run_lego_align(frame, dt))

        elif self.state == RobotState.INTAKE:
            self.transition(self.run_intake(now))

        elif self.state == RobotState.TURN_TO_LINE:
            self.transition(self.run_turn_to_line(frame))

        elif self.state == RobotState.LINE_FOLLOW_HOME:
            self.transition(self.run_line_follow(frame, dt, search_target=False))

        elif self.state == RobotState.RECOVERY:
            self.transition(self.run_recovery(frame))

        elif self.state == RobotState.STOP:
            self.drivebase.stop(coast=False)


def main():
    cam = OpenCVCamera()
    p = Perception(cam, False)
    drivebase = DriveBase()
    claw = Claw()

    target_aligner = AlignmentController(omega_max=0.18, x_tol=0.05)
    target_aligner.align_pd.update_params(kp=0.25, kd=0.0)

    approach_controller = ApproachController(
        omega_max=0.2,
        v_max=0.15,
        pickup_y=300,
        x_tol=0.08,
    )
    approach_controller.lateral_pd.update_params(kp=0.05, kd=0.02)

    line_follower = LineFollowingController(
        k_heading_slow=2,
        v_min=0.25,
        v_max=0.7,
        omega_max=0.15,
    )
    line_follower.lateral_pd.update_params(kp=0.2, kd=0.02)

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