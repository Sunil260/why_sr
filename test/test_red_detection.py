import cv2
from perception import OpenCVCamera, Perception

cam = OpenCVCamera()
perception = Perception(cam)

while True:

    frame = cam.get_frame()

    res = perception.detect_red_line(frame)

    if res.detected:
        print(
            f"x_center={res.x_error_center:.1f} "
            f"x_ahead={res.x_error_ahead:.1f} "
            f"heading={res.heading_error_ahead:.3f}"
        )

    cv2.imshow("Red Line Test", frame)

    if cv2.waitKey(1) == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()