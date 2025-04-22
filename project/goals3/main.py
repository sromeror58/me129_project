# Imports
import pigpio
import time
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
    TURN_L,
    HOOK_L,
    TURN_R,
    HOOK_R,
    SPIN_L,
    SPIN_R,
)
from sensor_estimation import (
    IntersectionEstimator,
    SideEstimator,
    EndOfStreetEstimator,
    NextStreetDetector,
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

    def pull_forward(self, intersection_estimator, threshold=0.5):
        """Performs the pull forward behavior after detecting a valid intersection.

        Args:
            intersection_estimator (IntersectionEstimator): IntersectionEstimator object
            threshold (float): Threshold for state changes (default 0.5, ~one time constant)
        """
        self.drive_system.stop()
        t_0 = intersection_estimator.tlast
        curr = time.time()
        while curr - t_0 <= threshold:
            self.drive_system.drive(STRAIGHT)
            curr = time.time()
        self.drive_system.stop()

    def turn_to_next_street(self, direction):
        """Performs the turning behavior to find the next street at an intersection.

        This behavior:
        1. Spins in the specified direction
        2. Detects when the robot has left the current street
        3. Detects when the robot has found the next street
        4. Stops when aligned with the next street

        Args:
            direction (str): Direction to turn, either 'left' or 'right'
        """
        # Validate and convert the direction parameter
        if direction.lower() == "left":
            spin_direction = SPIN_L
        elif direction.lower() == "right":
            spin_direction = SPIN_R
        else:
            print(f"Invalid direction: {direction}. Using 'left' as default.")
            spin_direction = SPIN_L

        print(f"Turning to next street: {direction}")

        # Initialize the next street detector
        next_street_detector = NextStreetDetector()

        # Continuous loop for turning behavior
        while True:
            # Read sensor data
            reading = self.sensors.read()

            if next_street_detector.update(reading, time_constant=0.05):
                print("Found and aligned with next street!")
                break

            else:
                self.drive_system.drive(spin_direction)

        self.drive_system.stop()

    def run_complex(self):
        """
        This runs a more complex path following task with the turn
        around aspect incorporated (goals 2, task 5.1)
        """

        # Initialize estimators
        intersection_estimator = IntersectionEstimator()
        side_estimator = SideEstimator()
        eos_estimator = EndOfStreetEstimator()

        tlast = time.time()
        intersection_estimator.tlast = tlast
        side_estimator.tlast = tlast
        eos_estimator.tlast = tlast

        while True:
            # Read sensor data
            reading = self.sensors.read()

            # Check for intersection
            if intersection_estimator.update(reading, 0.2):
                print("Intersection detected!")
                self.pull_forward(intersection_estimator)
                # Now turn to find the next street (default: left turn)
                self.turn_to_next_street("right")
                break

            # Estimate which side of the road the robot is on
            side = side_estimator.update(reading, 0.05)
            print(f"Road side: {side}")

            # Check for end of street
            if eos_estimator.update(reading, side, 0.1):
                print("End of street detected!")
                # self.stop()
                break

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
                # When all sensors read 0, use the road side estimator to decide what to do
                if side == SideEstimator.LEFT:
                    # If pushed to the left, turn or hook right to get back to center
                    self.drive_system.drive(HOOK_R)
                elif side == SideEstimator.RIGHT:
                    # If pushed to the right, turn or hook left to get back to center
                    self.drive_system.drive(HOOK_L)
                else:
                    # If centered, keep going straight
                    self.drive_system.drive(STRAIGHT)
            else:
                # considering the other 3 cases i.e. 101, 111, and 000
                self.drive_system.stop()

    def run_simple_brain(self):
        # Initialize estimators
        intersection_estimator = IntersectionEstimator()
        side_estimator = SideEstimator()
        eos_estimator = EndOfStreetEstimator()

        tlast = time.time()
        intersection_estimator.tlast = tlast
        side_estimator.tlast = tlast
        eos_estimator.tlast = tlast
        while True:
            try:
                cmd = input("Enter command (s, l, r, q): ").strip().lower()
            except KeyboardInterrupt:
                self.stop()
                break
            if cmd == "q":
                print("Quiting")
                self.stop()
                break
            elif cmd == "l":
                print("Turning left...")
                t0 = time.time()
                # perform left turn
                self.turn_to_next_street("left")
                t_end = time.time()
                print(t_end - t0)
            elif cmd == "r":
                print("Turning right...")
                t0 = time.time()
                # perform right turn
                self.turn_to_next_street("right")
                t_end = time.time()
                print(t_end - t0)
            elif cmd == "s":
                print("Going Straight")
                # go straght
                while True:
                    reading = self.sensors.read()
                    # Check for intersection
                    if intersection_estimator.update(reading, 0.2):
                        print("Intersection detected!")
                        self.pull_forward(intersection_estimator)
                        break

                    # Estimate which side of the road the robot is on
                    side = side_estimator.update(reading, 0.05)
                    print(f"Road side: {side}")

                    # Check for end of street
                    if eos_estimator.update(reading, side, 0.1):
                        print("End of street detected!")
                        self.drive_system.stop()
                        # self.stop()
                        break

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
                        # When all sensors read 0, use the road side estimator to decide what to do
                        if side == SideEstimator.LEFT:
                            # If pushed to the left, turn or hook right to get back to center
                            self.drive_system.drive(HOOK_R)
                        elif side == SideEstimator.RIGHT:
                            # If pushed to the right, turn or hook left to get back to center
                            self.drive_system.drive(HOOK_L)
                        else:
                            # If centered, keep going straight
                            self.drive_system.drive(STRAIGHT)
                    else:
                        # considering the other 3 cases i.e. 101, 111, and 000
                        self.drive_system.stop()
            else:
                print("Invalid command, quitting...")
                self.stop()
                break

    def stop(self):
        """Safely stop all motors and clean up the pigpio connection"""
        try:
            # First stop the motors through the drive system
            self.drive_system.stop()
        except Exception as e:
            print(f"Error stopping drive system: {e}")

        try:
            # Then cleanly stop the pigpio interface if still connected
            if hasattr(self.io, "connected") and self.io.connected:
                self.io.stop()
        except Exception as e:
            print(f"Error stopping pigpio: {e}")


def main_complex():
    """
    Initializes the io, and robot object to perform the more complex path
    following task (goals 2, task 5.1)
    """
    # Initialize the pigpio interface
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon. Is it running?")
        return

    robot = Robot(io)

    try:
        robot.run_complex()

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        try:
            # Shutdown cleanly only if still connected
            if io.connected:
                robot.stop()
        except Exception:
            # If any exception occurs during cleanup, just print it
            print("Error during cleanup")
            traceback.print_exc()


def main_simple_brain():
    # Initialize the pigpio interface
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon. Is it running?")
        return
    robot = Robot(io)
    try:
        robot.run_simple_brain()
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        try:
            # Shutdown cleanly only if still connected
            if io.connected:
                robot.stop()
        except Exception:
            # If any exception occurs during cleanup, just print it
            print("Error during cleanup")
            traceback.print_exc()


if __name__ == "__main__":
    main_simple_brain()
