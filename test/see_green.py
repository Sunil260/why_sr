import cv2 as cv
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception import OpenCVCamera, Perception


def main():
    cam = OpenCVCamera()
    p = Perception(cam, debug=True)

    print("Starting green detection test... Press 'q' to quit.")

    try:
        while True:
            frame = cam.get_frame()

            # --- run your function ---
            result = p.analyze_green(frame)

            # --- recreate mask for visualization ---
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            lower_green = np.array([35, 50, 40])
            upper_green = np.array([90, 255, 255])

            mask = cv.inRange(hsv, lower_green, upper_green)

            # --- show mask ---
            cv.imshow("Green Mask", mask)

            # --- draw detection on frame ---
            display = frame.copy()

            if result.detected:
                cx = int(result.centroid_x)
                cy = int(result.centroid_y)

                cv.circle(display, (cx, cy), 5, (0, 0, 255), -1)
                cv.putText(display,
                           f"e_x={result.error_x:.2f}",
                           (10, 30),
                           cv.FONT_HERSHEY_SIMPLEX,
                           0.6,
                           (0, 255, 0),
                           2)

                print(f"Detected | e_x={result.error_x:.2f}, area={result.area:.1f}")
            else:
                print("No green detected")

            cv.imshow("Green Detection", display)

            # --- exit ---
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cam.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()