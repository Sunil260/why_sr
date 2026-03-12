import cv2
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception

cam = OpenCVCamera()
p = Perception(cam)

while True:

    frame = cam.get_frame()

    res = p.analyze_target(frame)

    if res.detected:
        print(
            f"Target detected | error_x={res.error_x:.1f} area={res.area:.0f}"
        )

    cv2.imshow("Target Detection", frame)

    if cv2.waitKey(1) == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()