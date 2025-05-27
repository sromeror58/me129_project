from config import DX_DY_TABLE


def getTurnAngle(angle1: float, angle2: float, isLeft: bool = True):
    """
    Calculate the turn angle between two angles.

    Args:
        angle1: First angle in degrees
        angle2: Second angle in degrees
        isLeft: Boolean indicating if the turn is to the left (True) or right (False)

    Returns:
        float: The turn angle in degrees. Positive for left turns, negative for right turns.
    """
    # Normalize angles to [-180, 180] range
    print(f'Angle 1={angle1}, Angle 2={angle2}')
    angle1 = ((angle1 + 180) % 360) - 180
    angle2 = ((angle2 + 180) % 360) - 180

    # Calculate the difference
    diff = angle2 - angle1

    if isLeft:
        # For left turns, we want positive values
        # If diff is negative, we need to go the other way around
        if diff < 0:
            diff += 360
    else:
        # For right turns, we want negative values
        # If diff is positive, we need to go the other way around
        if diff > 0:
            diff -= 360

    return diff



class Pose:
    """
    A class representing a position and heading in a 2D grid.

    The heading is represented as an integer from 0 to 7, where:
    0: North
    1: Northeast
    2: East
    3: Southeast
    4: South
    5: Southwest
    6: West
    7: Northwest
    """

    def __init__(self, xinit: float = 0.0, yinit: float = 0.0, heading: int = 0):
        """
        Initialize a new Pose object.

        Args:
            xinit: Initial x-coordinate (default: 0.0)
            yinit: Initial y-coordinate (default: 0.0)
            heading: Initial heading direction (0-7, default: 0)
        """
        self.x = xinit
        self.y = yinit
        self.heading = heading

    def clone(self):
        """
        Create a deep copy of this Pose object.

        Returns:
            Pose: A new Pose object with the same x, y, and heading values.
        """
        return Pose(self.x, self.y, self.heading)

    def calcmove(self):
        """
        Update x and y coordinates for one travel step in the direction of the current heading.

        This method uses the DX_DY_TABLE to determine the change in x and y coordinates
        based on the current heading, then updates the position accordingly.
        """
        dx, dy = DX_DY_TABLE[self.heading]
        self.x += dx
        self.y += dy
        print(f"Position after move: ({self.x}, {self.y}, heading: {self.heading})")

    def calcturn(self, turnAngle: float, isLeft: bool):
        """
        Calculate and apply a turn based on the given angle.

        Args:
            turnAngle: The angle to turn in degrees
            isLeft: Boolean indicating if the turn is to the left (True) or right (False)
        """
        turn = int(round(turnAngle / 45))
        self.heading = (self.heading + turn) % 8
        print(f"Position after turn: ({self.x}, {self.y}, heading: {self.heading})")

    def calcuturn(self):
        """
        Calculate a U-turn by rotating the heading 180 degrees (4 steps in the 8-direction system).
        """
        self.heading = (self.heading + 4) % 8
        print(f"Position after U-turn: ({self.x}, {self.y}, heading: {self.heading})")
