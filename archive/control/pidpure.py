import cv2 as cv
import numpy as np
import time
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
            self.stop()

    def stop(self):
        self.en.value = 0
        self.in1.off()
        self.in2.off()

# ---------------- PID CLASS ----------------

class PID:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0, integral_limit=2000):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.out_min = out_min
        self.out_max = out_max
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def update(self, error, dt):
        if dt <= 0:
            return 0.0

        # Integral
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        # Derivative
        if self.first:
            derivative = 0.0
            self.first = False
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        u = max(self.out_min, min(self.out_max, u))
        return u

# ---------------- CAMERA ----------------

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

# ---------------- MOTORS ----------------

left  = DCMotorL298(in1=17, in2=27, en_pwm=19)
right = DCMotorL298(in1=22, in2=23, en_pwm=13)

# ---------------- RED MASK ----------------

red_lower1 = np.array([0, 100, 100])
red_upper1 = np.array([10, 255, 255])

red_lower2 = np.array([160, 100, 100])
red_upper2 = np.array([180, 255, 255])

# ---------------- CONTROL PARAMETERS ----------------

lookahead_y = 400
Ld = 100.0
base_speed = 0.2
wheelbase = 0.15

# Pure pursuit feedforward gain
k_pp = 1.0

# PID gains for image error
# Start with these, then tune
pid = PID(
    kp=0.002,
    ki=0.0000,
    kd=0.002,
    out_min=-0.35,
    out_max=0.35,
    integral_limit=1000
)

prev_time = time.monotonic()

# ---------------- MAIN LOOP ----------------

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv.resize(frame, (480, 480))
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    mask1 = cv.inRange(hsv, red_lower1, red_upper1)
    mask2 = cv.inRange(hsv, red_lower2, red_upper2)
    mask = cv.bitwise_or(mask1, mask2)

    now = time.monotonic()
    dt = now - prev_time
    prev_time = now

    # -------- FIND LOOKAHEAD POINT --------

    row = mask[lookahead_y]
    xs = np.where(row > 0)[0]

    if len(xs) > 0:
        target_x = int(xs.mean())
        center_x = frame.shape[1] // 2
        error = float(target_x - center_x)   # pixels

        # --- Pure pursuit style feedforward ---
        kappa = 2.0 * error / (Ld * Ld)
        steer_ff = k_pp * kappa

        # --- PID feedback on image error ---
        steer_pid = pid.update(error, dt)

        # total steering command
        steer = steer_ff + steer_pid

        # Differential drive mixing
        left_speed  = base_speed - steer
        right_speed = base_speed + steer

        # Clamp to valid PWM range
        left_speed  = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        left.set(left_speed)
        right.set(right_speed)

        # draw lookahead point
        cv.circle(frame, (target_x, lookahead_y), 8, (0, 255, 0), -1)
        cv.line(frame, (center_x, lookahead_y), (target_x, lookahead_y), (255, 0, 0), 2)

        cv.putText(frame, f"err: {error:.1f}", (20, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv.putText(frame, f"pid: {steer_pid:.3f}", (20, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv.putText(frame, f"ff: {steer_ff:.3f}", (20, 90),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv.putText(frame, f"L: {left_speed:.2f} R: {right_speed:.2f}", (20, 120),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    else:
        # no line detected
        left.stop()
        right.stop()
        pid.reset()

    # cv.imshow("frame", frame)
    # cv.imshow("mask", mask)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
left.stop()
right.stop()