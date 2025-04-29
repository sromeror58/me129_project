import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import math

class Pose:
    def __init__(self, xinit: float = 0.0, yinit: float = 0.0, heading: int = 0):
        self.x = xinit
        self.y = yinit 
        self.heading = heading  

        self.dx_dy_table = [
            (0, 1),   # Heading 0: North
            (-1, 1),  # Heading 1: Northwest
            (-1, 0),  # Heading 2: West
            (-1, -1), # Heading 3: Southwest
            (0, -1),  # Heading 4: South
            (1, -1),  # Heading 5: Southeast
            (1, 0),   # Heading 6: East
            (1, 1)    # Heading 7: Northeast
        ]

    def clone(self):
        """Create a deep copy of this Pose object"""
        return Pose(self.x, self.y, self.heading)

    def calcmove(self):
        """
        Update x and y for one travel step in the direction of the current heading.
        """
        dx, dy = self.dx_dy_table[self.heading]
        self.x += dx
        self.y += dy
        print(f"Position after move: ({self.x}, {self.y}, heading: {self.heading})")

    def calcturn(self, angle1: float, angle2: float):
        """
        Calculate the turn angle between two angles.
        
        Args:
            angle1: First angle in degrees (north is +/- 180, CCW increases from -180 to 180)
            angle2: Second angle in degrees 
            
        Returns:
            float: Turn angle in degrees. Positive for left turns (CCW), negative for right turns (CW).
        """
        # Normalize angles to [-180, 180] range
        angle1 = ((angle1 + 180) % 360) - 180
        angle2 = ((angle2 + 180) % 360) - 180
        diff = angle2 - angle1
        # Handle case where we need to go the other way around the circle
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        turn = int(round(diff / 45))
        self.heading = (self.heading + turn) % 8
        print(f"Position after turn: ({self.x}, {self.y}, heading: {self.heading})")

    def calcuturn(self):
        self.heading = (self.heading + 4) % 8
        print(f"Position after U-turn: ({self.x}, {self.y}, heading: {self.heading})")


