import cv2
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception

cam = OpenCVCamera()
p = Perception(cam,True)
lw_dected = False
while True:

    frame = cam.get_frame()
    output = p.detect_target_cheap(frame)


    if lw_dected is not True:
        if output.detected:
            print(f"found w {output.bpx} px" f"output.detected={output.detected}")
            lw_dected = True
            cv2.destroyAllWindows()
        else:
            print(f"Not FOUND w {output.bpx} px" f"output.detected={output.detected}")
    
    if (lw_dected):
        result = p.analyze_target(frame)
        print(
            f"here"
        )

    if cv2.waitKey(1) == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()
p.close_debug()