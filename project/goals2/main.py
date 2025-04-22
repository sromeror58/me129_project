# Imports
import pigpio
import traceback
from drive_system import DriveSystem
from sensor import LineSensor
from motor import Motor
from config import (
    PIN_MOTOR1_LEGA,
    PIN_MOTOR1_LEGB,
    PIN_MOTOR2_LEGA,
    PIN_MOTOR2_LEGB,
    PIN_IR_LEFT,
    PIN_IR_MIDDLE,
    PIN_IR_RIGHT,
    STRAIGHT,
    TURN_R,
    HOOK_R,
    TURN_L,
    HOOK_L,
    SPIN_L,
)


class Robot:
    """
    Class that encapsulates all the robot components such as the drive system
    and sensors.
    """

    def __init__(self, io):
        """Initializes the motors, drive system and sensors.

        Args:
            io (pigpio.pi): pigpio interface instance
        """
        self.io = io
        motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, io, 1000)
        motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, io, 1000)
        self.drive_system = DriveSystem(io, motor1, motor2)
        self.sensors = LineSensor(io, PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT)

    def run_basic(self):
        """
        This runs the most basic path following task (goals 2, task 4)
        """
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
        """
        This runs a more complex path following task with the turn
        around aspect incorporated (goals 2, task 5.1)
        """
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

    def stop(self):
        self.drive_system.stop()
        self.io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
        self.io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
        self.io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
        self.io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)
        self.io.stop()


def main_basic():
    """
    Initializes the io, and robot object to perform the most basic path
    following task (goals 2, task 4)
    """
    io = pigpio.pi()
    robot = Robot(io)
    try:
        robot.run_basic()
    except Exception as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    # Shutdown cleanly (stopping the motors, then stopping the io).
    robot.drive_system.drive.stop()
    robot.io.stop()


def main_complex():
    """
    Initializes the io, and robot object to perform the more complex path
    following task (goals 2, task 5.1)
    """
    io = pigpio.pi()
    robot = Robot(io)
    try:
        robot.run_complex()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down robot.")
        robot.stop()
    except Exception as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    # Shutdown cleanly (stopping the motors, then stopping the io).
    robot.stop()


if __name__ == "__main__":
    main_complex()
