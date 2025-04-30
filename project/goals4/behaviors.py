import time
from sensor_estimation import (
    IntersectionEstimator,
    SideEstimator,
    EndOfStreetEstimator,
    NextStreetDetector,
)
from pose import getTurnAngle
from config import (
    STRAIGHT,
    TURN_L,
    HOOK_L,
    TURN_R,
    HOOK_R,
    SPIN_L,
    SPIN_R,
)


class Behaviors:
    """
    Class that encapsulates the robot's high-level behaviors.

    This class provides methods for executing complex robot behaviors such as
    turning, line following, and intersection handling. It coordinates between
    the drive system, sensors, and magnetometer to implement these behaviors.
    """

    def __init__(self, drive_system, sensors, adc):
        """
        Initialize the Behaviors class with the necessary components.

        Args:
            drive_system: The robot's drive system for motor control
            sensors: The robot's sensors for environmental perception
            adc: The analog-to-digital converter for magnetometer readings
        """
        self.drive_system = drive_system
        self.sensors = sensors
        self.adc = adc

    def turn_to_next_street(self, direction):
        """
        Performs the turning behavior to find the next street at an intersection.

        This behavior:
        1. Spins in the specified direction
        2. Detects when the robot has left the current street
        3. Detects when the robot has found the next street
        4. Stops when aligned with the next street
        5. Calculates the turn angle based on magnetometer readings

        Args:
            direction (str): Direction to turn, either 'left' or 'right'

        Returns:
            float: The calculated turn angle in degrees
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
        t0 = time.time()
        angle1 = self.adc.readangle()
        adjustedangle1 = None

        while True:
            # Read sensor data
            reading = self.sensors.read()

            state, isTransition = next_street_detector.update(
                reading, time_constant=0.085
            )

            if isTransition:
                adjustedangle1 = self.adc.readangle()
                print("BING")

            if state:
                curr = time.time()
                isLeft = False
                if spin_direction == SPIN_L:
                    isLeft = True
                print(
                    f"Found and aligned with next street!\nTime to turn to next street was {curr - t0} sec."
                )
                break

            else:
                self.drive_system.drive(spin_direction)

        self.drive_system.stop()

        turnAngle1 = getTurnAngle(angle1, self.adc.readangle(), isLeft)
        print(turnAngle1)
        if adjustedangle1:
            turnAngle2 = getTurnAngle(adjustedangle1, self.adc.readangle(), isLeft)
            print(turnAngle2)

            # If less than full turn, since full turn's are pretty obvious
            if abs(turnAngle2 - turnAngle1) <= 90:
                # Want to weight closer to turnAngle2 the larger the turn is: 0/5 to 4/5
                x = abs(turnAngle1) / 360
                weight = (3 * x**2 - 2 * x**3) / 1.1
                turnAngle1 = (1 - weight) * turnAngle1 + (weight) * turnAngle2

        print(turnAngle1)
        return turnAngle1

    def pull_forward(self, travel_time=0.5):
        """
        Performs the pull forward behavior after detecting a valid intersection.

        This method drives the robot straight ahead for a specified duration,
        typically used to center the robot in an intersection or move past
        a detected intersection point.

        Args:
            travel_time (float): Amount of time in seconds to pull forward.
                               Defaults to 0.5 seconds.
        """
        t_0 = time.time()
        curr = time.time()
        while curr - t_0 <= travel_time:
            self.drive_system.drive(STRAIGHT)
            curr = time.time()
        self.drive_system.stop()

    def line_follow(self):
        """
        Performs the line following behavior.

        This method implements the core line-following algorithm that:
        1. Continuously reads sensor data to determine the robot's position relative to the line
        2. Detects intersections and handles them appropriately
        3. Detects the end of streets and initiates U-turns
        4. Adjusts the robot's direction based on sensor readings to stay on the line

        Returns:
            tuple: (isUturn, travel_time) where:
                - isUturn (bool): True if a U-turn was performed, False otherwise
                - travel_time (float): Time spent following the line in seconds
        """
        intersection_estimator = IntersectionEstimator()
        side_estimator = SideEstimator()
        eos_estimator = EndOfStreetEstimator()

        t0 = time.time()
        while True:
            reading = self.sensors.read()

            # Check for intersection
            if intersection_estimator.update(reading, 0.1):
                curr = time.time()
                # Then pull forward
                self.pull_forward(travel_time=0.36)

                # isUturn, travel time
                return False, curr - t0

            # Estimate which side of the road the robot is on
            side = side_estimator.update(reading, 0.05)
            print(f"Road side: {side}")

            # Check for end of street
            if eos_estimator.update(reading, side, 0.09):
                self.pull_forward(travel_time=0.6)
                curr = time.time()
                print("End of street detected!")
                self.turn_to_next_street("left")
                self.line_follow()
                self.drive_system.stop()

                # isUturn, travel time of street, one way
                return True, curr - t0

            if reading == (0, 1, 0):
                self.drive_system.drive(STRAIGHT)
            elif reading == (1, 1, 1):
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
