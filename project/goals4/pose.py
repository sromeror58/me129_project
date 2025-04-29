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

    def calcmove(self):
        """
        Update x and y for one travel step in the direction of the current heading.
        """
        dx, dy = self.dx_dy_table[self.heading]
        self.x += dx
        self.y += dy
        print(f"Position after move: ({self.x}, {self.y}, heading: {self.heading})")
        self.show()

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
        self.show()

    def calcuturn(self):
        self.heading = (self.heading + 4) % 8
        print(f"Position after U-turn: ({self.x}, {self.y}, heading: {self.heading})")
        self.show()

    def show(self):
        """
        Show the x/y/heading of the current robot pose.
        """
        # Clear the current, or create a new figure.
        plt.clf()
        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-3.5, 3.5)
        plt.gca().set_ylim(-3.5, 3.5)
        plt.gca().set_aspect('equal')
        # Show all the possible locations.
        for x in range(-3, 4):
            for y in range(-3, 4):
                plt.plot(x, y, color='lightgray', marker='o', markersize=8)
        # Get the direction vector for the current heading
        dx, dy = self.dx_dy_table[self.heading]
        
        # Scale the direction vector to length 0.5
        arrow_length = 0.5
        scale = arrow_length / math.sqrt(dx*dx + dy*dy)
        dx *= scale
        dy *= scale
        
        # Calculate start and end points to center the arrow
        x_start = self.x - dx/2  # Start point offset by half the arrow length
        y_start = self.y - dy/2
        x_end = self.x + dx/2    # End point offset by half the arrow length
        y_end = self.y + dy/2
        
        # Draw the arrow
        plt.arrow(x_start, y_start,          # Start position (base)
                 dx, dy,                      # Direction and length
                 width=0.2,                  # Width of arrow body
                 head_width=0.3,             # Width of arrow head
                 head_length=0.1,            # Length of arrow head
                 color='magenta')            # Color of the arrow
        plt.pause(0.001)
