from rpi_hardware_pwm import HardwarePWM
import time

# Pi 5: GPIO18 => pwm_channel=2 with dtoverlay=pwm-2chan
pwm = HardwarePWM(pwm_channel=2, hz=50, chip=0)

def set_pulse_us(pulse_us: int):
    duty = (pulse_us / 20000.0) * 100.0  # 50Hz => 20ms => 20000us
    pwm.change_duty_cycle(duty)

CENTER_US = 1570  # your calibrated stop

try:
    pwm.start((CENTER_US / 20000.0) * 100.0)  # start at center

    # test motion
    set_pulse_us(CENTER_US + 100)  # one direction
    time.sleep(1)
    set_pulse_us(CENTER_US)        # stop
    time.sleep(0.5)

    set_pulse_us(CENTER_US - 100)  # other direction
    time.sleep(1)
    set_pulse_us(CENTER_US)        # stop
    time.sleep(0.5)

finally:
    pwm.stop()