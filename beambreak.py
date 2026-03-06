from gpiozero import Button
from ServoPwmDriver import ContinuousServoPWM
import time

class BeamBreak:
    def __init__(self, pin, debounce):
        self.sensor = Button(pin, pull_up=True)
        
        self.is_broken = False

        self.sensor.when_pressed = self._beam_broken
        self.sensor.when_released = self._beam_restored

    def _beam_broken(self):
        self.is_broken = True
        print("broken")
    
    def _beam_restored(self):
        self.is_broken = False
        print("restored")
    
    def beam_state(self):
        return self.is_broken

    def wait_press(self):
        self.sensor.wait_for_press()


if __name__ == "__main__":
    beam = BeamBreak(24, 0.01)
    servo = ContinuousServoPWM(pwm_channel=2, chip=0, center_us=1500)


    try:

        print(beam.beam_state())
        beam.wait_press()

        if(beam.beam_state()):
            servo.ccw(0.85, delta_us=100)
            time.sleep(4)
        

        servo.cw(0.85, delta_us=100) 
        time.sleep(0.5)

    finally:
        servo.stop()
        pass