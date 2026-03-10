import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

# -------- GREEN HSV RANGE --------
green_lower = np.array([40, 80, 80])
green_upper = np.array([85, 255, 255])

lookahead_y = 320

while True:

    ret, frame = cap.read()
    if not ret:
        break

    frame = cv.resize(frame, (480,480))

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    mask = cv.inRange(hsv, green_lower, green_upper)

    # ---- detect line in a row ----
    row = mask[lookahead_y]
    xs = np.where(row > 0)[0]

    if len(xs) > 0:

        target_x = int(xs.mean())
        center_x = frame.shape[1] // 2
        error = target_x - center_x

        print("Green line detected at X:", target_x, " Error:", error)

        # draw detection
        cv.circle(frame, (target_x, lookahead_y), 8, (0,255,0), -1)

    else:
        print("No green line detected")

    cv.imshow("frame", frame)
    cv.imshow("mask", mask)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()