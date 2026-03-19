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



def main():

    cam = OpenCVCamera()
    p = Perception(cam,False)
    lw_detected = False
    aligned = False
    target_aligner = AlignmentController()
    approach_targer = ApproachController()
    drivebase = DriveBase()
    # From red_line_follow.py setting same controller values
    line_follower = LineFollowingController(k_heading_slow=0, v_min=0.25, v_max=0.7, omega_max=0.15)
    line_follower.lateral_pd.update_params(kp=0.2, kd=0.02)



    try:
        prev_t = time.monotonic()

        while True:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frame = cam.get_frame()
            
            if lw_detected is not True:


                output = p.detect_target_cheap(frame)
                if output.detected:
                    print(f"found w {output.bpx} px" f"output.detected={output.detected}")
                    lw_detected = True
                    drivebase.stop()
                    cv2.destroyAllWindows()
                    continue #skip rest of loop
                else:
                    print(f"Not FOUND w {output.bpx} px" f"output.detected={output.detected}")
            
# khush updates, need review --> moving to a separate file
            # if lw_detected is not True:

            #     output = p.detect_target_cheap(frame)
            #     if output.detected:
            #         print(f"found w {output.bpx} px" f"output.detected={output.detected}")
            #         lw_detected = True
            #         drivebase.stop()
            #         cv2.destroyAllWindows()
            #         continue #skip rest of loop
            #     else:
            #         print(f"Not FOUND w {output.bpx} px" f"output.detected={output.detected}")
# end of khush updates
            
            if (lw_detected):
                
                if not aligned:
                    
                    result = p.analyze_target(frame)
                    # TargetEstimate(detected=True, centroid_x=red_cx, centroid_y=red_cy, area=blue_w * blue_h, error_x=error_x, error_y=error_y)
                    if result.detected:
                        print(
                            f"{result.detected} "
                            f"e_x = {np.round(result.error_x,2 )} "
                            f"e_y = {np.round(result.error_y,2)} "
                            f"Blue area = {np.round(result.area,2)} "
                        )
                        if abs(result.error_x) < target_aligner.x_tol:
                            aligned = True

                        command = target_aligner.compute(result, dt)
                        print(command.v, command.omega)
                        drivebase.set_Velocity(command.v , command.omega)
                else: 
                    #approach control
                    result = p.detect_legoman(frame)
                    if result.detected:
                        print(
                            f"{result.detected} "
                            f"e_x = {np.round(result.e_x,2 )} "
                            f"e_y = {np.round(result.e_y,2)} "
                            f"head area =  {np.round(result.area,2)} "
                        )
    
    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        drivebase.stop()
        cam.release()
        p.close_debug()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()