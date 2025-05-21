import pigpio
import traceback
import time
from motor import Motor
from config import PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, PIN_MOTOR2_LEGA, PIN_MOTOR2_LEGB


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
        "STRAIGHT": (0.705, 0.78),
        "VEER_L": (0.73, 0.77),
        "STEER_L": (0.62, 0.77),
        "TURN_L": (0.47, 0.79),
        "HOOK_L": (0.0, 0.8),
        "SPIN_L": (-0.81, 0.76),
        "VEER_R": (0.79, 0.67),
        "STEER_R": (0.79, 0.60),
        "TURN_R": (0.82, 0.45),
        "HOOK_R": (0.83, 0),
        "SPIN_R": (0.72, -0.82),
    }

    def __init__(self, io):
        """
        Initialize the drive system with two motors.

        Args:
            io (pigpio.pi): pigpio interface for GPIO control
        """
        self.io = io
        self.motor1 = Motor(PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, self.io, 1000)
        self.motor2 = Motor(PIN_MOTOR2_LEGB, PIN_MOTOR2_LEGA, self.io, 1000)
        # Convert drive style tuples to DriveValues objects
        self.drive_styles = {}
        for style, values in self.DRIVE_STYLES.items():
            self.drive_styles[style] = DriveValues(*values)

    def stop(self):
        """Stop both motors."""
        self.motor1.stop()
        self.motor2.stop()

    def drive(self, style: str, backwards=False):
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
        if backwards:
            self.motor1.setlevel(-1 * drive_val.level1)
            self.motor2.setlevel(-1 * drive_val.level2)
        else:
            self.motor1.setlevel(drive_val.level1)
            self.motor2.setlevel(drive_val.level2)
    
    def pwm(self, pwm_l, pwm_r):
        self.motor1.setlevel(pwm_l)
        self.motor2.setlevel(pwm_r)
        pass


if __name__ == "__main__":
    # Testing different drive modes
    try:
        # Initialize pigpio and motors
        io = pigpio.pi()

        # Create drive system and test a drive style
        ds = DriveSystem(io)
        ds.drive("STRAIGHT")
        time.sleep(4)
        ds.stop()

        io.stop()

    except BaseException as e:
        print("Ending due to exception: %s" % repr(e))
        traceback.print_exc()
