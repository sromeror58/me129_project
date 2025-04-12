# Imports
import pigpio
import sys
import time
import traceback
from drive_system import DriveSystem
from sensor import LineSensor
from motor import Motor

PIN_MOTOR1_LEGA = 8
PIN_MOTOR1_LEGB = 7
PIN_MOTOR2_LEGA = 5
PIN_MOTOR2_LEGB = 6

# Define the IR pins.
PIN_IR_LEFT   = 14      # Default GPIO Channel for Left   IR Detector
PIN_IR_MIDDLE = 15      # Default GPIO Channel for Middle IR Detector
PIN_IR_RIGHT  = 18      # Default GPIO Channel for Right  IR Detector

STRAIGHT = "STRAIGHT"
VEER_L = "VEER_L"
STEER_L = "STEER_L"
VEER_R = "VEER_R"
STEER_R = "STEER_R"
TURN_L = "TURN_L"
HOOK_L = "HOOK_L"
TURN_R = "TURN_R"
HOOK_R = "HOOK_R"
SPIN_L = "SPIN_L"



class Robot():
    def __init__(self, io):
        self.io = io
        motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, io, 1000)
        motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, io, 1000)
        self.drive_system = DriveSystem(io, motor1, motor2)
        self.sensors = LineSensor(io, PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT)
    
    def run_basic(self):
        while True:
            reading = self.sensors.read()
            if reading == (0, 1, 0):
                self.drive_system.drive(STRAIGHT)
            elif reading == (0, 1, 1):
                self.drive_system.drive(TURN_R)
            elif reading == (0, 0, 1):
                self.drive_system.drive(HOOK_R)
            elif reading == (1, 1, 0):
                self.drive_system.drive(TURN_L)
            elif reading == (1, 0, 0):
                self.drive_system.drive(HOOK_L)
            else:
                # considering the other 3 cases i.e. 101, 111, and 000
                self.drive_system.stop()
    def run_complex(self):
        while True:
            reading = self.sensors.read()
            if reading == (0, 1, 0):
                self.drive_system.drive(STRAIGHT)
            elif reading == (0, 1, 1):
                self.drive_system.drive(TURN_R)
            elif reading == (0, 0, 1):
                self.drive_system.drive(HOOK_R)
            elif reading == (1, 1, 0):
                self.drive_system.drive(TURN_L)
            elif reading == (1, 0, 0):
                self.drive_system.drive(HOOK_L)
            elif reading == (0, 0, 0):
                self.drive_system.drive(SPIN_L)
            else:
                # considering the other 3 cases i.e. 101, 111, and 000
                self.drive_system.stop()

def main_basic():
    io = pigpio.pi()
    robot = Robot(io)
    try:
        robot.run_basic()
    except:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    # Shutdown cleanly (stopping the motors, then stopping the io).
    drive.stop()
    io.stop()

def main_complex():
    io = pigpio.pi()
    robot = Robot(io)
    try:
        robot.run_complex()
    except:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    # Shutdown cleanly (stopping the motors, then stopping the io).
    drive.stop()
    io.stop()


if __name__ == "__main__":
    main_complex()