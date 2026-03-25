'''
Class for the drive base of the robot
- 2 DC motors driven by L298N H-bridge driver boards
'''

from hardware.dc_motor_driver import DCMotorL298

class DriveBase:

    def __init__(self, left_pins=(17, 27, 19), right_pins=(22, 23, 13)):
        self.left_motor = DCMotorL298(*left_pins)
        self.right_motor = DCMotorL298(*right_pins)

    def set_Velocity(self,linear_speed, angular_speed):
        #Set speed for both motors. Speeds in range of [-1.0, 1.0].
        left_speed = linear_speed - angular_speed
        right_speed = linear_speed + angular_speed

        # Apply Trim to balance them
        # left_final = left_raw
        # right_final = right_raw * motor_trim (depends on which) test w straight line

        self.left_motor.set(left_speed)
        self.right_motor.set(right_speed)

    def stop(self, coast=True):
        #coast motors
        self.left_motor.stop(coast=coast)
        self.right_motor.stop(coast=coast)
    

    def close(self):
        # stop motors and release resources
        self.stop()