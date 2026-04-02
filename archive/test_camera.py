import cv2
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera

cam = OpenCVCamera()

print("Press q to quit")

while True:

    frame = cam.get_frame()

    cv2.imshow("Camera Test", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()