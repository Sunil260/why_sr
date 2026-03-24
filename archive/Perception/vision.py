import time
import cv2 as cv
import numpy as np

class RedLineDetector:
    def __init__(self, cam_index=0, w=640, h=480, roi_height=220):
        self.cap = cv.VideoCapture(cam_index)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH,  w)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv.CAP_PROP_FPS, 30)

        self.w, self.h = w, h
        self.roi_height = roi_height

        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])

    def read_pose(self):
        ret, frame = self.cap.read()
        if not ret:
            return False, 0.0, 0.0, None, None

        frame = cv.resize(frame, (self.w, self.h))

        y0 = self.h - self.roi_height
        roi = frame[y0:self.h, :]

        hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        m1 = cv.inRange(hsv, self.red_lower1, self.red_upper1)
        m2 = cv.inRange(hsv, self.red_lower2, self.red_upper2)
        mask = cv.bitwise_or(m1, m2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, 0.0, frame, mask

        c = max(contours, key=cv.contourArea)
        if cv.contourArea(c) < 200:
            return False, 0.0, 0.0, frame, mask

        pts = c.reshape(-1, 2).astype(np.float32)
        line = cv.fitLine(pts, cv.DIST_L2, 0, 0.01, 0.01)

        
        vx, vy, x0_fit, y0_fit = line.flatten()
        vx, vy, x0_fit, y0_fit = float(vx), float(vy), float(x0_fit), float(y0_fit)

        angle = np.arctan2(vy, vx)
        theta_e = angle - (np.pi / 2)
        theta_e = (theta_e + np.pi) % (2*np.pi) - np.pi

        y_target = self.roi_height - 1
        if abs(vy) < 1e-6:
            x_at_bottom = x0_fit
        else:
            t = (y_target - y0_fit) / vy
            x_at_bottom = x0_fit + t * vx

        center_x = self.w / 2
        e_px = x_at_bottom - center_x
        e_norm = float(e_px / center_x)

        return True, e_norm, theta_e, frame, mask

    def release(self):
        self.cap.release()


def ascii_mask(mask, width=64, height=24):
    """Terminal preview of a binary mask."""
    small = cv.resize(mask, (width, height), interpolation=cv.INTER_AREA)
    on = small > 0
    return "\n".join(
        "".join("#" if on[y, x] else " " for x in range(width))
        for y in range(height)
    )


if __name__ == "__main__":
    det = RedLineDetector(cam_index=0, w=640, h=480, roi_height=220)
    last_print = 0.0

    try:
        while True:
            found, e_norm, theta_e, _, mask = det.read_pose()

            now = time.time()
            if now - last_print > 0.2 and mask is not None:  # ~5 Hz
                last_print = now
                print("\x1b[2J\x1b[H", end="")  # clear screen
                print(ascii_mask(mask, 64, 24))
                if found:
                    print(f"\nFOUND  e_norm={e_norm:+.3f}  theta_e={theta_e:+.3f} rad")
                else:
                    print("\nNOT FOUND")

    finally:
        det.release()