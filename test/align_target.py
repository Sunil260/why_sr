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

