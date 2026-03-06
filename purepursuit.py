import cv2 as cv
import numpy as np
from gpiozero import DigitalOutputDevice, PWMOutputDevice

# ---------------- MOTOR CLASS ----------------

class DCMotorL298:
    def __init__(self, in1, in2, en_pwm, pwm_freq=1000):
        self.in1 = DigitalOutputDevice(in1)
        self.in2 = DigitalOutputDevice(in2)
        self.en  = PWMOutputDevice(en_pwm, frequency=pwm_freq)
        self.stop()

    def set(self, speed):
        speed = max(-1.0, min(1.0, float(speed)))

        if speed > 0:
            self.in1.on()
            self.in2.off()
            self.en.value = speed

        elif speed < 0:
            self.in1.off()
            self.in2.on()
            self.en.value = -speed

        else:
            self.en.value = 0

    def stop(self):
        self.en.value = 0
        self.in1.off()
        self.in2.off()

# ---------------- CAMERA ----------------

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

# ---------------- MOTORS ----------------

left  = DCMotorL298(in1=17, in2=27, en_pwm=19)
right = DCMotorL298(in1=22, in2=23, en_pwm=13)

# ---------------- RED MASK ----------------

red_lower1 = np.array([0,100,100])
red_upper1 = np.array([10,255,255])

red_lower2 = np.array([160,100,100])
red_upper2 = np.array([180,255,255])

# ---------------- PURE PURSUIT PARAMETERS ----------------

lookahead_y = 300
Ld = 150
base_speed = 0.4
wheelbase = 0.15

# ---------------- MAIN LOOP ----------------

while True:

    ret, frame = cap.read()
    if not ret:
        break

    frame = cv.resize(frame,(480,480))

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    mask1 = cv.inRange(hsv, red_lower1, red_upper1)
    mask2 = cv.inRange(hsv, red_lower2, red_upper2)
    mask = cv.bitwise_or(mask1, mask2)

    # -------- FIND LOOKAHEAD POINT --------

    row = mask[lookahead_y]

    xs = np.where(row > 0)[0]

    if len(xs) > 0:

        target_x = int(xs.mean())

        center_x = frame.shape[1] // 2
        error = target_x - center_x

        # PURE PURSUIT CURVATURE
        kappa = 2 * error / (Ld * Ld)

        # WHEEL SPEEDS
        left_speed  = base_speed * (1 - kappa * wheelbase/2)
        right_speed = base_speed * (1 + kappa * wheelbase/2)

        left.set(left_speed)
        right.set(right_speed)

        # draw lookahead point
        cv.circle(frame,(target_x,lookahead_y),8,(0,255,0),-1)

    else:
        # no line detected
        left.stop()
        right.stop()

    cv.imshow("frame",frame)
    cv.imshow("mask",mask)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()

left.stop()
right.stop()