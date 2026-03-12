import cv2
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