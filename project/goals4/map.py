from enum import Enum
from pose import Pose
import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import math

class STATUS(Enum):
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4

class Intersection:
    def __init__(self, x, y):
        """
        Initialize an intersection with coordinates and street statuses.
        
        Args:
            x (float): Longitude coordinate
            y (float): Latitude coordinate
        """
        self.x = x
        self.y = y
        self.streets = [STATUS.UNKNOWN] * 8


    def updateStreet(self, heading, status):
        if self.streets[heading] not in [STATUS.UNKNOWN, STATUS.UNEXPLORED]:
            return
        self.streets[heading] = status


class Map:
    def __init__(self, x=0.0, y=0.0, heading=0):
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
        self.intersections = {}
        self.getintersection(x, y)

    def getintersection(self, x, y):
        """
        Get an intersection at the specified coordinates, creating it if it doesn't exist.
        
        Args:
            x (float): Longitude coordinate
            y (float): Latitude coordinate
            
        Returns:
            Intersection: The intersection at the specified coordinates
        """
        if (x, y) not in self.intersections:
            self.intersections[(x, y)] = Intersection(x, y)
        return self.intersections[(x, y)] 


    def outcomeA(self, x0, y0, h0, x1, y1, h1, isLeft: bool):
        """
        Set streets to STATUS.NONEXISTENT based on the direction of turn from h0 to h1.
        
        Args:
            x0 (float): Starting x coordinate
            y0 (float): Starting y coordinate
            h0 (int): Starting heading (0-7)
            x1 (float): Ending x coordinate
            y1 (float): Ending y coordinate
            h1 (int): Ending heading (0-7)
            isLeft (bool): True if turning left, False if turning right
        """
        intersection = self.getintersection(x0, y0)

        if isLeft:
            # Left turn
            if h0 < h1:
                for h in range(h0 + 1, h1):
                    intersection.updateStreet(h, STATUS.NONEXISTENT)
            else:  
                for h in range(h0 + 1, 8):
                    intersection.updateStreet(h, STATUS.NONEXISTENT)
                for h in range(0, h1):
                    intersection.updateStreet(h, STATUS.NONEXISTENT)
        else:  
            # Right turn
            if h1 < h0:
                for h in range(h1 + 1, h0):
                    intersection.updateStreet(h, STATUS.NONEXISTENT)
            else:  
                for h in range(h1 + 1, 8):
                    intersection.updateStreet(h, STATUS.NONEXISTENT)
                for h in range(0, h0):
                    intersection.updateStreet(h, STATUS.NONEXISTENT)

        intersection.updateStreet(h1, STATUS.UNEXPLORED)
        # self.plot(pose1)

    def outcomeB(self, x0, y0, h0, x1, y1, h1):
        intersection = self.getintersection(x0, y0)
        intersection.streets[h0] = STATUS.CONNECTED
        intersection = self.getintersection(x1, y1)
        intersection.streets[(h1 + 4) % 8] = STATUS.CONNECTED
        # self.plot(pose1)

    def outcomeC(self, x0, y0, h0, x1, y1, h1):
        intersection = self.getintersection(x0, y0)
        if intersection.streets[h0] != STATUS.DEADEND:
            intersection.streets[h0] = STATUS.DEADEND
            intersection = self.getintersection(x1, y1)
            intersection.streets[(h0 + 4) % 8] = STATUS.CONNECTED
            for i in range(0, 8):
                if i != h0:
                    intersection.streets[i] = STATUS.NONEXISTENT
        # self.plot(pose1)

    def plot(self, x, y, heading):
        """
        Show the x/y/heading of the current robot pose.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            heading (int): Heading (0-7)
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
        dx, dy = self.dx_dy_table[heading]
        
        # Scale the direction vector to length 0.5
        arrow_length = 0.5
        scale = arrow_length / math.sqrt(dx*dx + dy*dy)
        dx *= scale
        dy *= scale
        
        # Calculate start and end points to center the arrow
        x_start = x - dx/2  # Start point offset by half the arrow length
        y_start = y - dy/2
        x_end = x + dx/2    # End point offset by half the arrow length
        y_end = y + dy/2
        
        # Draw the arrow
        plt.arrow(x_start, y_start,          # Start position (base)
                 dx, dy,                     # Direction and length
                 width=0.2,                  # Width of arrow body
                 head_width=0.3,             # Width of arrow head
                 head_length=0.1,            # Length of arrow head
                 color='magenta')            # Color of the arrow

        # Draw the intersections
        # Define colors for each status
        status_colors = {
            STATUS.UNKNOWN: 'black',
            STATUS.NONEXISTENT: 'lightgray', 
            STATUS.UNEXPLORED: 'blue',
            STATUS.DEADEND: 'red',
            STATUS.CONNECTED: 'green'
        }

        # For each intersection in the map
        for (x,y) in self.intersections:
            # For each possible heading from that intersection
            for h in range(8):
                # Get the direction vector for this heading
                dx, dy = self.dx_dy_table[h]
                
                # Scale to half-length (0.5 for cardinal, ~0.707 for diagonal)
                length = 0.5 if dx == 0 or dy == 0 else 0.3 * math.sqrt(2)
                dx *= length
                dy *= length
                
                # Get intersection status and corresponding color
                status = self.getintersection(x,y).streets[h]
                color = status_colors[status]
                
                # Draw line from intersection halfway to next
                plt.plot([x, x + dx], [y, y + dy], color=color)

        plt.pause(0.001)