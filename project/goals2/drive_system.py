import pigpio
import traceback
import time
from motor import Motor


class DriveValues:
    """
    Stores motor power levels for both motors in a drive system.
    """

    def __init__(self, level1: float, level2: float):
        """
        Initialize with power levels for both motors.

        Args:
            level1 (float): Power level for motor 1 (-1.0 to 1.0)
            level2 (float): Power level for motor 2 (-1.0 to 1.0)
        """
        self.level1 = level1
        self.level2 = level2


class DriveSystem:
    """
    Controls a two-motor drive system with predefined driving styles.

    Provides methods to drive the robot in various patterns including
    straight, veering, turning, and spinning in both directions.
    """

    # Predefined motor power levels for different driving styles
    DRIVE_STYLES = {
        "STRAIGHT": (0.76, 0.719),
        "VEER_L": (0.73, 0.77),
        "STEER_L": (0.62, 0.77),
        "TURN_L": (0.47, 0.79),
        "HOOK_L": (0.0, 0.8),
        "SPIN_L": (-0.7, 0.65),
        "VEER_R": (0.79, 0.67),
        "STEER_R": (0.79, 0.60),
        "TURN_R": (0.82, 0.45),
        "HOOK_R": (0.83, 0),
        "SPIN_R": (0.71, -0.69),
    }

    def __init__(self, io, motor1: Motor, motor2: Motor):
        """
        Initialize the drive system with two motors.

        Args:
            io (pigpio.pi): pigpio interface for GPIO control
            motor1 (Motor): First motor object (typically left motor)
            motor2 (Motor): Second motor object (typically right motor)
        """
        self.io = io
        self.motor1 = motor1
        self.motor2 = motor2
        # Convert drive style tuples to DriveValues objects
        self.drive_styles = {}
        for style, values in self.DRIVE_STYLES.items():
            self.drive_styles[style] = DriveValues(*values)

    def stop(self):
        """Stop both motors."""
        self.motor1.stop()
        self.motor2.stop()

    def drive(self, style: str):
        """
        Drive the robot using a predefined driving style.

        Args:
            style (str): Name of the driving style to use
                        (must be one of the keys in DRIVE_STYLES)

        Raises:
            ValueError: If the specified style is not recognized
        """
        if style not in self.drive_styles:
            raise ValueError(f"Invalid drive style: {style}")
        drive_val = self.drive_styles[style]
        self.motor1.setlevel(drive_val.level1)
        self.motor2.setlevel(drive_val.level2)


if __name__ == "__main__":
    # Testing different drive modes
    try:
        # GPIO pin definitions for motors
        PIN_MOTOR1_LEGA = 8
        PIN_MOTOR1_LEGB = 7
        PIN_MOTOR2_LEGA = 5
        PIN_MOTOR2_LEGB = 6

        # Initialize pigpio and motors
        io = pigpio.pi()
        motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, io, 1000)
        motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, io, 1000)

        # Create drive system and test a drive style
        ds = DriveSystem(io, motor1, motor2)
        ds.drive("SPIN_R")
        time.sleep(4)
        ds.stop()

        # Clean up: clear the PWM pins
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
        io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
        io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)

        io.stop()

    except BaseException as e:
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()
