'''
beam break sensor as a button with pull up resistor
- using gpiozero Button class for debouncing and event handling
- methods:
    - beam_state(): returns True if beam is currently broken, False if intact
    - wait_for_object(): blocks until beam is broken (object detected)
    
'''
from gpiozero import Button
import time

class BeamBreak:
    def __init__(self, pin, debounce):
        
        self.sensor = Button(pin, pull_up=True, bounce_time=debounce)
        self.is_broken = False
        self.sensor.when_pressed = self._beam_broken #triggers a broken beam (pull up, so high to low)
        self.sensor.when_released = self._beam_restored # triggers a restored beam (pull up, so low to high)

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

    try:

        print(beam.beam_state())
        beam.wait_press()

        if(beam.beam_state()):
            time.sleep(4)
        print(beam.beam_state())
        time.sleep(0.5)

    finally:

        pass