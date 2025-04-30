from enum import Enum
from pose import Pose
import matplotlib
import os
# Check if display_map environment variable is set to "on"
import matplotlib.pyplot as plt
import math
from config import DX_DY_TABLE

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
            
        if os.environ.get('display_map') == 'on':
            matplotlib.use('TkAgg')
        else:
            matplotlib.use('Agg')

    def updateStreet(self, heading, status):
        if self.streets[heading] not in [STATUS.UNKNOWN, STATUS.UNEXPLORED]:
            return
        self.streets[heading] = status


class Map:
    def __init__(self, pose=None):
        self.intersections = {}
        if pose is None:
            pose = Pose()
        self.getintersection(pose.x, pose.y)

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


    def outcomeA(self, pose0, pose1, isLeft: bool):
        """
        Set streets to STATUS.NONEXISTENT based on the direction of turn from pose0 to pose1.
        
        Args:
            pose0 (Pose): Starting pose
            pose1 (Pose): Ending pose
            isLeft (bool): True if turning left, False if turning right
        """
        intersection = self.getintersection(pose0.x, pose0.y)
        h0 = pose0.heading
        h1 = pose1.heading

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
        self.plot(pose1)

    def outcomeB(self, pose0, pose1):
        """
        Update intersection status for a straight movement from pose0 to pose1.
        
        Args:
            pose0 (Pose): Starting pose
            pose1 (Pose): Ending pose
        """
        intersection = self.getintersection(pose0.x, pose0.y)
        intersection.streets[pose0.heading] = STATUS.CONNECTED
        intersection = self.getintersection(pose1.x, pose1.y)
        intersection.streets[(pose1.heading + 4) % 8] = STATUS.CONNECTED
        self.plot(pose1)

    def outcomeC(self, pose0, pose1):
        """
        Update intersection status for a U-turn from pose0 to pose1.
        
        Args:
            pose0 (Pose): Starting pose
            pose1 (Pose): Ending pose
        """
        intersection = self.getintersection(pose0.x, pose0.y)
        intersection.streets[pose0.heading] = STATUS.DEADEND
        self.plot(pose1)

    def plot(self, pose):
        """
        Create a plot showing the current robot pose.
        
        Args:
            pose (Pose): Current robot pose
        """
        # Clear the current, or create a new figure.
        plt.clf()
        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-3.5, 3.5)
        plt.gca().set_ylim(-3.5, 3.5)
        plt.gca().set_aspect('equal')
        # Show all the possible locations.
        for x_grid in range(-3, 4):
            for y_grid in range(-3, 4):
                plt.plot(x_grid, y_grid, color='lightgray', marker='o', markersize=8)
        # Get the direction vector for the current heading
        dx, dy = DX_DY_TABLE[pose.heading]
        
        # Scale the direction vector to length 0.5
        arrow_length = 0.5
        scale = arrow_length / math.sqrt(dx*dx + dy*dy)
        dx *= scale
        dy *= scale
        
        # Calculate start and end points to center the arrow
        x_start = pose.x - dx/2  # Start point offset by half the arrow length
        y_start = pose.y - dy/2
        x_end = pose.x + dx/2    # End point offset by half the arrow length
        y_end = pose.y + dy/2
        
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
        for (x_int, y_int) in self.intersections:
            # For each possible heading from that intersection
            for h in range(8):
                # Get the direction vector for this heading
                dx, dy = DX_DY_TABLE[h]
                
                # Scale to half-length (0.5 for cardinal, ~0.707 for diagonal)
                length = 0.5 if dx == 0 or dy == 0 else 0.3 * math.sqrt(2)
                dx *= length
                dy *= length
                
                # Get intersection status and corresponding color
                status = self.getintersection(x_int, y_int).streets[h]
                color = status_colors[status]
                
                # Draw line from intersection halfway to next
                plt.plot([x_int, x_int + dx], [y_int, y_int + dy], color=color)
        
        # If using TkAgg backend, display the plot
        if os.environ.get('display_map') == 'on':
            try:
                plt.draw()
                plt.pause(0.1)  # Small pause to allow the plot to update
            except KeyboardInterrupt:
                # Silently handle keyboard interrupts during plotting
                pass

    def save_plot(self, pose):
        """
        Create the plot and save it to a file instead of displaying it.
        
        Args:
            pose (Pose): Current robot pose
        """
        self.plot(pose)
        
        # Instead of plt.pause(), save the figure to a file
        filename = f"plots/map_x{pose.x}_y{pose.y}_h{pose.heading}.png"
        plt.savefig(filename)
        print(f"Saved map to {filename}")
        plt.close()
        
    def close(self):
        """
        Properly close all matplotlib resources.
        This should be called when the program exits.
        """
        try:
            plt.close('all')  # Close all figures
            if os.environ.get('display_map') == 'on':
                plt.close()  # Ensure the main figure is closed
        except KeyboardInterrupt:
            # Silently handle keyboard interrupts during cleanup
            pass
