import cv2
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception

cam = OpenCVCamera()
p = Perception(cam)

try:
    while True:
        frame = cam.get_frame()
        res = p.detect_green_box(frame)

        if res.detected:
            print(
                f"Green box detected | area={res.area:.0f} error_x={res.error_x}"
            )

        # Skip GUI if no monitor
        # cv2.imshow("Green Box Test", frame)
        # if cv2.waitKey(1) == ord("q"):
        #     break

except KeyboardInterrupt:
    print("Test stopped by user")

finally:
    cam.release()
    cv2.destroyAllWindows()