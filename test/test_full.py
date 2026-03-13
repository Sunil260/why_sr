import cv2
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception

cam = OpenCVCamera()
p = Perception(cam)

while True:

    frame = cam.get_frame()

    line = p.detect_red_line(frame)
    target = p.analyze_target(frame)
    safe = p.detect_green_box(frame)

    if line.detected:
        print("Line detected")

    if target.detected:
        print("Target detected")

    if safe.detected:
        print("Safe zone detected")

    cv2.imshow("Perception", frame)

    if cv2.waitKey(1) == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()