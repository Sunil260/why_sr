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
    def __init__(self, pin, hold):
        
        self.sensor = Button(pin, pull_up=False, hold_time=hold)
        self.is_missing = True
        self.sensor.when_pressed = self._beam_found #triggers a broken beam (pull up, so high to low)
        self.sensor.when_released = self._beam_missing # triggers a restored beam (pull up, so low to high)

    def _beam_found(self):
        self.is_missing = False
        print("found")
    
    def _beam_missing(self):
        self.is_missing = True
        print("missing")
    
    def beam_state(self):
        return self.is_missing

    def wait_press(self):
        self.sensor.wait_for_press()

if __name__ == "__main__":
    beam = BeamBreak(24, 0.2)

    try:

        print(beam.beam_state())
        #beam.wait_press()
        while(beam.beam_state()):
            time.sleep(3)

        #if(beam.beam_state()):
        #    time.sleep(4)
        #print(beam.beam_state())
        time.sleep(0.5)

    finally:

        pass