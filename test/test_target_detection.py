import cv2
import sys
import os
import time
import numpy as np
from collections import deque

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception
from controllers import AlignmentController
from drivebase import DriveBase


X_TOL = 0.02           # normalized
OMEGA_MAX = 0.20
KP_ALIGN = 1
KD_ALIGN = 0.0

USE_SMOOTHING = True
SMOOTH_WINDOW = 5

USE_MIN_TURN = True
MIN_TURN_OMEGA = 0.2


def main():
    cam = OpenCVCamera()
    p = Perception(cam, True)
    drivebase = DriveBase()

    target_aligner = AlignmentController(omega_max=OMEGA_MAX, x_tol=X_TOL)
    target_aligner.align_pd.update_params(kp=KP_ALIGN, kd=KD_ALIGN)

    error_hist = deque(maxlen=SMOOTH_WINDOW)

    try:
        prev_t = time.monotonic()

        while True:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frame = cam.get_frame()
            result = p.analyze_target(frame)

            if not result.detected:
                print("Target not detected")
                drivebase.stop(coast=False)

            else:
                error_x = result.error_x

                if USE_SMOOTHING:
                    error_hist.append(error_x)
                    error_x = float(np.mean(error_hist))
                else:
                    error_hist.clear()

                print(
                    f"detected={result.detected} "
                    f"error_x={error_x:+.3f} "
                    f"error_y={result.error_y:+.3f} "
                    f"area={result.area:.1f}"
                )

                result.error_x = error_x

                if abs(result.error_x) < target_aligner.x_tol:
                    print("Aligned -> stop")
                    drivebase.stop(coast=False)

                else:
                    cmd = target_aligner.compute(result, dt)
                    omega = cmd.omega

                    # Only use this if the motor deadband is a real problem.
                    # Keep it small or it will overshoot near center.
                    if USE_MIN_TURN and abs(omega) > 0.01 and abs(result.error_x) > 3 * target_aligner.x_tol:
                        omega = np.sign(omega) * max(abs(omega), MIN_TURN_OMEGA)

                    omega = max(-target_aligner.omega_max, min(target_aligner.omega_max, omega))

                    print(f"command: v=0.00 omega={omega:+.3f}")
                    drivebase.set_Velocity(0.0, omega)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        drivebase.stop()
        cam.release()
        p.close_debug()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()