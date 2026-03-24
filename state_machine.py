'''
Brain of the S&R robot
- State machine to manage the different states of the robot and the transitions between them
- various controllers and perception modules will be used in different states to achieve the desired behavior
'''

from enum import Enum, auto
from dataclasses import dataclass
import time

from controllers import (
    DriveCommand,
    LineEstimate,
    TargetEstimate,
    SafeZoneEstimate,
)


class RobotState(Enum):
    INIT = auto()
    ACQUIRE_LINE = auto()

    FOLLOW_PATH_TO_TARGET = auto()
    ALIGN_TO_TARGET = auto()
    APPROACH_TARGET = auto()
    PICKUP_TARGET = auto()

    FOLLOW_PATH_TO_SAFE_ZONE = auto()
    ALIGN_TO_SAFE_ZONE = auto()
    APPROACH_SAFE_ZONE = auto()
    DROP_TARGET = auto()

    FOLLOW_PATH_HOME = auto()
    STOP = auto()
    RECOVERY = auto()


@dataclass
class PerceptionBundle:
    line: LineEstimate
    target: TargetEstimate
    safe_zone: SafeZoneEstimate


class StateMachine:
    def __init__(
        self,
        drivebase,
        claw,
        line_controller,
        align_controller,
        approach_controller,
        turn_until_line_controller,
    ):
        self.drivebase = drivebase
        self.claw = claw

        self.line_controller = line_controller
        self.align_controller = align_controller
        self.approach_controller = approach_controller
        self.turn_until_line_controller = turn_until_line_controller

        self.state = RobotState.INIT
        self.prev_state = None

        self.state_start_time = time.monotonic()
        self.retry_count = 0
        self.max_retries = 3

        self.has_object = False
        self.mission_complete = False

        # Detection debouncing / filtering
        self.target_detect_count = 0
        self.safe_zone_detect_count = 0
        self.line_detect_count = 0

        self.target_detect_threshold = 3
        self.safe_zone_detect_threshold = 3
        self.line_detect_threshold = 2

        # Tunable thresholds
        self.target_align_tol_px = 20
        self.safe_zone_align_tol_px = 20

        self.target_area_approach_threshold = 3500
        self.safe_zone_area_approach_threshold = 5000

        self.pickup_time_s = 0.6
        self.drop_time_s = 0.6

        self.recovery_turn_direction = 1.0  # +1 or -1

    # ---------- Utility ----------

    def reset_detection_counters(self):
        self.target_detect_count = 0
        self.safe_zone_detect_count = 0
        self.line_detect_count = 0

    def reset_controllers(self):
        # Add reset() methods to your task controllers if needed
        if hasattr(self.line_controller, "lateral_pd"):
            self.line_controller.lateral_pd.reset()

        if hasattr(self.align_controller, "align_pd"):
            self.align_controller.align_pd.reset()

        if hasattr(self.approach_controller, "heading_pd"):
            self.approach_controller.heading_pd.reset()

    def transition_to(self, new_state: RobotState):
        self.prev_state = self.state
        self.state = new_state
        self.state_start_time = time.monotonic()
        self.reset_controllers()
        self.reset_detection_counters()
        print(f"[FSM] {self.prev_state} -> {self.state}")

    def time_in_state(self) -> float:
        return time.monotonic() - self.state_start_time

    def stop_robot(self):
        self.drivebase.set_velocity(0.0, 0.0)

    def apply_drive_command(self, cmd: DriveCommand):
        self.drivebase.set_velocity(cmd.v, cmd.omega)

    def target_seen_consistently(self, target_est: TargetEstimate) -> bool:
        if target_est.detected:
            self.target_detect_count += 1
        else:
            self.target_detect_count = 0
        return self.target_detect_count >= self.target_detect_threshold

    def safe_zone_seen_consistently(self, safe_est: SafeZoneEstimate) -> bool:
        if safe_est.detected:
            self.safe_zone_detect_count += 1
        else:
            self.safe_zone_detect_count = 0
        return self.safe_zone_detect_count >= self.safe_zone_detect_threshold

    def line_seen_consistently(self, line_est: LineEstimate) -> bool:
        if line_est.detected:
            self.line_detect_count += 1
        else:
            self.line_detect_count = 0
        return self.line_detect_count >= self.line_detect_threshold

    # ---------- Main FSM update ----------

    def update(self, perception: PerceptionBundle, dt: float):
        """
        Call this every loop.
        It decides what command to send to the drivebase / claw.
        """

        line_est = perception.line
        target_est = perception.target
        safe_est = perception.safe_zone

        # ---------------- INIT ----------------
        if self.state == RobotState.INIT:
            self.stop_robot()
            self.transition_to(RobotState.ACQUIRE_LINE)
            return

        # ---------------- ACQUIRE_LINE ----------------
        elif self.state == RobotState.ACQUIRE_LINE:
            if self.line_seen_consistently(line_est):
                self.transition_to(RobotState.FOLLOW_PATH_TO_TARGET)
                self.stop_robot()
                return

            if self.time_in_state() > 5.0:
                self.transition_to(RobotState.RECOVERY)
                return

            cmd = self.turn_until_line_controller.compute(line_est, dt)
            self.apply_drive_command(cmd)
            return

        # ---------------- FOLLOW_PATH_TO_TARGET ----------------
        elif self.state == RobotState.FOLLOW_PATH_TO_TARGET:
            if not line_est.detected:
                if self.time_in_state() > 1.0:
                    self.transition_to(RobotState.RECOVERY)
                    return

            if self.target_seen_consistently(target_est):
                self.transition_to(RobotState.ALIGN_TO_TARGET)
                self.stop_robot()
                return

            cmd = self.line_controller.compute(line_est, dt)
            self.apply_drive_command(cmd)
            return

        # ---------------- ALIGN_TO_TARGET ----------------
        elif self.state == RobotState.ALIGN_TO_TARGET:
            if not target_est.detected:
                if self.time_in_state() > 1.0:
                    self.transition_to(RobotState.FOLLOW_PATH_TO_TARGET)
                    return

            if target_est.detected and abs(target_est.error_x) <= self.target_align_tol_px:
                self.transition_to(RobotState.APPROACH_TARGET)
                self.stop_robot()
                return

            if self.time_in_state() > 4.0:
                self.transition_to(RobotState.RECOVERY)
                return

            cmd = self.align_controller.compute(target_est, dt)
            self.apply_drive_command(cmd)
            return

        # ---------------- APPROACH_TARGET ----------------
        elif self.state == RobotState.APPROACH_TARGET:
            if not target_est.detected:
                if self.time_in_state() > 1.0:
                    self.transition_to(RobotState.ALIGN_TO_TARGET)
                    return

            # Option 1: beam break confirms pickup position reached
            # Option 2: area threshold means target is close enough
            if self.claw.is_object_detected() or (
                target_est.detected and target_est.area >= self.target_area_approach_threshold
            ):
                self.transition_to(RobotState.PICKUP_TARGET)
                self.stop_robot()
                return

            if self.time_in_state() > 5.0:
                self.transition_to(RobotState.RECOVERY)
                return

            cmd = self.approach_controller.compute(target_est, dt)
            self.apply_drive_command(cmd)
            return

        # ---------------- PICKUP_TARGET ----------------
        elif self.state == RobotState.PICKUP_TARGET:
            self.stop_robot()

            # Very simple timed pickup logic
            if self.time_in_state() < 0.05:
                self.claw.close()

            if self.time_in_state() > self.pickup_time_s:
                if self.claw.is_object_detected():
                    self.has_object = True
                    self.transition_to(RobotState.FOLLOW_PATH_TO_SAFE_ZONE)
                else:
                    # Failed pickup
                    self.retry_count += 1
                    if self.retry_count > self.max_retries:
                        self.transition_to(RobotState.RECOVERY)
                    else:
                        self.transition_to(RobotState.ALIGN_TO_TARGET)
                return

            return

        # ---------------- FOLLOW_PATH_TO_SAFE_ZONE ----------------
        elif self.state == RobotState.FOLLOW_PATH_TO_SAFE_ZONE:
            if not line_est.detected:
                if self.time_in_state() > 1.0:
                    self.transition_to(RobotState.RECOVERY)
                    return

            if self.safe_zone_seen_consistently(safe_est):
                self.transition_to(RobotState.ALIGN_TO_SAFE_ZONE)
                self.stop_robot()
                return

            cmd = self.line_controller.compute(line_est, dt)
            self.apply_drive_command(cmd)
            return

        # ---------------- ALIGN_TO_SAFE_ZONE ----------------
        elif self.state == RobotState.ALIGN_TO_SAFE_ZONE:
            if not safe_est.detected:
                if self.time_in_state() > 1.0:
                    self.transition_to(RobotState.FOLLOW_PATH_TO_SAFE_ZONE)
                    return

            if safe_est.detected and abs(safe_est.error_x) <= self.safe_zone_align_tol_px:
                self.transition_to(RobotState.APPROACH_SAFE_ZONE)
                self.stop_robot()
                return

            if self.time_in_state() > 4.0:
                self.transition_to(RobotState.RECOVERY)
                return

            # reuse alignment controller
            cmd = self.align_controller.compute(safe_est, dt)
            self.apply_drive_command(cmd)
            return

        # ---------------- APPROACH_SAFE_ZONE ----------------
        elif self.state == RobotState.APPROACH_SAFE_ZONE:
            if not safe_est.detected:
                if self.time_in_state() > 1.0:
                    self.transition_to(RobotState.ALIGN_TO_SAFE_ZONE)
                    return

            if safe_est.detected and safe_est.area >= self.safe_zone_area_approach_threshold:
                self.transition_to(RobotState.DROP_TARGET)
                self.stop_robot()
                return

            if self.time_in_state() > 5.0:
                self.transition_to(RobotState.RECOVERY)
                return

            # Reuse approach controller if it supports SafeZoneEstimate too
            cmd = self.approach_controller.compute(safe_est, dt)
            self.apply_drive_command(cmd)
            return

        # ---------------- DROP_TARGET ----------------
        elif self.state == RobotState.DROP_TARGET:
            self.stop_robot()

            if self.time_in_state() < 0.05:
                self.claw.open()

            if self.time_in_state() > self.drop_time_s:
                self.has_object = False
                self.transition_to(RobotState.FOLLOW_PATH_HOME)
                return

            return

        # ---------------- FOLLOW_PATH_HOME ----------------
        elif self.state == RobotState.FOLLOW_PATH_HOME:
            if not line_est.detected:
                if self.time_in_state() > 1.0:
                    self.transition_to(RobotState.RECOVERY)
                    return

            # Placeholder:
            # Replace this with your actual "home reached" logic,
            # e.g. start marker detection / wide horizontal line / timed return / odometry flag.
            home_reached = False

            if home_reached:
                self.transition_to(RobotState.STOP)
                self.stop_robot()
                return

            cmd = self.line_controller.compute(line_est, dt)
            self.apply_drive_command(cmd)
            return

        # ---------------- RECOVERY ----------------
        elif self.state == RobotState.RECOVERY:
            # Simple fallback behavior:
            # rotate slowly until line is found again
            if line_est.detected:
                if self.has_object:
                    self.transition_to(RobotState.FOLLOW_PATH_TO_SAFE_ZONE)
                else:
                    self.transition_to(RobotState.FOLLOW_PATH_TO_TARGET)
                return

            if self.time_in_state() > 6.0:
                self.transition_to(RobotState.STOP)
                return

            self.apply_drive_command(DriveCommand(v=0.0, omega=0.6 * self.recovery_turn_direction))
            return

        # ---------------- STOP ----------------
        elif self.state == RobotState.STOP:
            self.stop_robot()
            self.mission_complete = True
            return