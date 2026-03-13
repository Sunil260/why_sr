
import time
from PID import PID
from hardware.dc_motor_driver import DCMotorL298
# (Assuming VelocityRamp is in its own file or top of main)

# Configuration
FINAL_TARGET_MS = 0.3  # Where we want to end up
ACCEL_RATE = 0.2      # m/s^2 (Takes 1.5 seconds to reach 0.3m/s)
DT = 0.05             # 20Hz Loop

# Init Hardware & Controllers
motor_l = DCMotorL298(17, 27, 12)
motor_r = DCMotorL298(22, 23, 13)

pid_l = PID(kp=0.6, ki=0.3, kd=0.02) # Adjusted Kp for smoother pickup
pid_r = PID(kp=0.6, ki=0.3, kd=0.02)
ramp = VelocityRamp(accel_rate=ACCEL_RATE)

pwm_l, pwm_r = 0.0, 0.0

def main():

    try:
        print(f"Ramping to {FINAL_TARGET_MS} m/s...")
        while True:
            # 1. Update the moving Target (The "Ramp")
            active_target = ramp.update(FINAL_TARGET_MS, DT)

            # 2. Get current real-world velocity
            v_l = motor_l.get_velocity_ms(DT)
            v_r = motor_r.get_velocity_ms(DT)

            # 3. PID computes the adjustment to PWM
            pwm_l += pid_l.compute(active_target, v_l, DT)
            pwm_r += pid_r.compute(active_target, v_r, DT)

            # 4. Apply safety-clipped PWM
            motor_l.set_speed(max(0, min(pwm_l, 1.0)))
            motor_r.set_speed(max(0, min(pwm_r, 1.0)))

            if active_target < FINAL_TARGET_MS:
                print(f"Ramping... Target: {active_target:.2f} | L: {v_l:.2f} | R: {v_r:.2f}")
            
            time.sleep(DT)

    except KeyboardInterrupt:
        motor_l.stop()
        motor_r.stop()
    


if __name__ == "__main__":
    main()
