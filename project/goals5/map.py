from enum import Enum
from pose import Pose
import matplotlib
import os
import pickle

# Check if display_map environment variable is set to "on"
import matplotlib.pyplot as plt
import math
from config import DX_DY_TABLE


def sortedInsert(list, node):
    """
    Inserts an Intersection node into a list of Intersection objects, maintaining ascending order by cost.
    Args:
        list (list of Intersection): The list of nodes sorted by increasing cost.
        node (Intersection): The node to insert into the list.
    """
    # Scan through the list, comparing costs
    for i in range(len(list)):
        if list[i].cost > node.cost:
        # Found the proper place to insert.
            list.insert(i, node)
            return
    # Nothing found, append at end.
    list.append(node)
    return

def distance(dx, dy):
    if (dx, dy) in [(1,0), (0, 1), (-1, 0), (0, -1)]:
        return 1.4
    return 1

class STATUS(Enum):
    """
    Enumeration representing the possible states of a street in the map.

    Values:
        UNKNOWN: Street status is not yet determined
        NONEXISTENT: Street does not exist at this intersection
        UNEXPLORED: Street exists but has not been fully explored
        DEADEND: Street leads to a dead end
        CONNECTED: Street connects to another intersection
    """

    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4


class Intersection:
    """
    Represents an intersection in the map with coordinates and street statuses.

    Each intersection has 8 possible streets (one for each direction) and tracks
    the status of each street.
    """

    def __init__(self, x, y, streets=None):
        """
        Initialize an intersection with coordinates and street statuses.

        Args:
            x (float): Longitude coordinate
            y (float): Latitude coordinate
        """
        self.x = x
        self.y = y
        self.cost = float('inf')
        self.direction = None
        if streets is not None:
            self.streets = streets
        else:
            self.streets = [STATUS.UNKNOWN] * 8

        if os.environ.get("display_map") == "on":
            matplotlib.use("TkAgg")
        else:
            matplotlib.use("Agg")

    def updateStreet(self, heading, status):
        """
        Update the status of a street at the specified heading.

        Only updates the status if the current status is UNKNOWN or UNEXPLORED.

        Args:
            heading (int): Direction of the street (0-7)
            status (STATUS): New status to set for the street
        """
        if self.streets[heading] not in [STATUS.UNKNOWN, STATUS.UNEXPLORED]:
            return
        self.streets[heading] = status
    def setcost(self, cost):
        self.cost = cost


class Map:
    """
    Represents a map of intersections and their connections.

    The map tracks the robot's exploration of a grid of intersections,
    recording which streets exist, which have been explored, and which
    lead to dead ends or connect to other intersections.
    """

    def __init__(self, pose=None, intersection=None):
        """
        Initialize a new map with an optional starting pose.

        Args:
            pose (Pose, optional): Starting position and heading of the robot.
                                  Defaults to a new Pose at (0,0) facing north.
            intersection (dictionary, optional): Initial dictionary of intersection
                                                mapping (x,y) to an Intersection object.
        """
        if intersection is None:
            self.intersections = {}
        else:
            self.intersections = intersection
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

        This method is called when the robot makes a turn. It marks all streets between
        the starting heading and ending heading as non-existent, except for the street
        that was actually taken.

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

    def outcomeB(self, pose0, pose1, road_ahead=False):
        """
        Update intersection status for a straight movement from pose0 to pose1.

        This method is called when the robot moves straight ahead. It marks
        the street as connected at both the starting and ending intersections.

        Args:
            pose0 (Pose): Starting pose
            pose1 (Pose): Ending pose
        """
        intersection = self.getintersection(pose0.x, pose0.y)
        intersection.streets[pose0.heading] = STATUS.CONNECTED
        intersection = self.getintersection(pose1.x, pose1.y)
        intersection.streets[(pose1.heading + 4) % 8] = STATUS.CONNECTED
        if road_ahead:
            intersection.updateStreet(pose1.heading, STATUS.UNEXPLORED)
        else:
            intersection.updateStreet(pose1.heading, STATUS.NONEXISTENT)
        # intersection.streets[pose1.heading] = STATUS.UNEXPLORED if road_ahead else STATUS.NONEXISTENT
        self.plot(pose1)

    def outcomeC(self, pose0, pose1):
        """
        Update intersection status for a U-turn from pose0 to pose1.

        This method is called when the robot makes a U-turn. It marks
        the street as a dead end at the starting intersection.

        Args:
            pose0 (Pose): Starting pose
            pose1 (Pose): Ending pose
        """
        intersection = self.getintersection(pose0.x, pose0.y)
        intersection.streets[pose0.heading] = STATUS.DEADEND
        self.plot(pose1)

    def plot(self, pose):
        """
        Create a plot showing the current robot pose and map state.

        This method visualizes the map, showing all intersections, streets,
        and their statuses, along with the current robot position and heading.

        Args:
            pose (Pose): Current robot pose
        """
        # Clear the current, or create a new figure.
        plt.clf()
        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-3.5, 3.5)
        plt.gca().set_ylim(-3.5, 3.5)
        plt.gca().set_aspect("equal")
        # Show all the possible locations.
        for x_grid in range(-3, 4):
            for y_grid in range(-3, 4):
                plt.plot(x_grid, y_grid, color="lightgray", marker="o", markersize=8)
        # Get the direction vector for the current heading
        dx, dy = DX_DY_TABLE[pose.heading]

        # Scale the direction vector to length 0.5
        arrow_length = 0.5
        scale = arrow_length / math.sqrt(dx * dx + dy * dy)
        dx *= scale
        dy *= scale

        # Calculate start and end points to center the arrow
        x_start = pose.x - dx / 2  # Start point offset by half the arrow length
        y_start = pose.y - dy / 2

        # Draw the arrow
        plt.arrow(
            x_start,
            y_start,  # Start position (base)
            dx,
            dy,  # Direction and length
            width=0.2,  # Width of arrow body
            head_width=0.3,  # Width of arrow head
            head_length=0.1,  # Length of arrow head
            color="magenta",
        )  # Color of the arrow

        # Draw the intersections
        # Define colors for each status
        status_colors = {
            STATUS.UNKNOWN: "black",
            STATUS.NONEXISTENT: "lightgray",
            STATUS.UNEXPLORED: "blue",
            STATUS.DEADEND: "red",
            STATUS.CONNECTED: "green",
        }

        # For each intersection in the map
        for x_int, y_int in self.intersections:
            # For each possible heading from that intersection
            for h in range(8):
                # Get the direction vector for this heading
                dx, dy = DX_DY_TABLE[h]
                dx *= 0.5
                dy *= 0.5

                # Get intersection status and corresponding color
                status = self.getintersection(x_int, y_int).streets[h]
                color = status_colors[status]

                # Draw line from intersection halfway to next
                plt.plot([x_int, x_int + dx], [y_int, y_int + dy], color=color)

        # If using TkAgg backend, display the plot
        if os.environ.get("display_map") == "on":
            try:
                plt.draw()
                plt.pause(0.1)  # Small pause to allow the plot to update
            except KeyboardInterrupt:
                # Silently handle keyboard interrupts during plotting
                pass
    
    def plot(self):
        """
        Create a plot showing the current map state only (FOR DIJKSTRA's).

        This method visualizes the map, showing all intersections, streets,
        and their statuses.
        """
        # Clear the current, or create a new figure.
        plt.clf()
        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-3.5, 3.5)
        plt.gca().set_ylim(-3.5, 3.5)
        plt.gca().set_aspect("equal")
        # Show all the possible locations.
        for x_grid in range(-3, 4):
            for y_grid in range(-3, 4):
                plt.plot(x_grid, y_grid, color="lightgray", marker="o", markersize=8)

        # Draw the intersections
        # Define colors for each status
        status_colors = {
            STATUS.UNKNOWN: "black",
            STATUS.NONEXISTENT: "lightgray",
            STATUS.UNEXPLORED: "blue",
            STATUS.DEADEND: "red",
            STATUS.CONNECTED: "green",
        }

        # For each intersection in the map
        for x_int, y_int in self.intersections:
            # For each possible heading from that intersection
            for h in range(8):
                # Get the direction vector for this heading
                dx, dy = DX_DY_TABLE[h]
                dx *= 0.5
                dy *= 0.5

                # Get intersection status and corresponding color
                status = self.getintersection(x_int, y_int).streets[h]
                color = status_colors[status]

                # Draw line from intersection halfway to next
                plt.plot([x_int, x_int + dx], [y_int, y_int + dy], color=color)

        # If using TkAgg backend, display the plot
        if os.environ.get("display_map") == "on":
            try:
                plt.draw()
                plt.pause(0.1)  # Small pause to allow the plot to update
            except KeyboardInterrupt:
                # Silently handle keyboard interrupts during plotting
                pass

    def save_map(self, filename='mymap.pickle'):
        """
        Create the plot and save it to a file instead of displaying it.

        This method is useful for saving map visualizations to files for later review
        or for generating reports.
        """
        self.plot(pose)

        # Instead of plt.pause(), save the figure to a file
        # filename = f"plots/map_x{pose.x}_y{pose.y}_h{pose.heading}.pickle"
        print("Saving the map to %s..." % filename)
        with open (filename, 'wb') as file:
            pickle.dump(self, file)
        # plt.savefig(filename)
        print(f"Saved map to {filename}")
    
    def load_map(self, filename="mymap.pickle"):
        # filename = f"plots/map_x{pose.x}_y{pose.y}_h{pose.heading}.pickle"
        print("Loading the map from %s..." % filename) 
        with open(filename, 'rb') as file:
            map = pickle.load(file)
        print(f"Loaded map")

    def close(self):
        """
        Properly close all matplotlib resources.

        This should be called when the program exits to ensure all
        matplotlib resources are properly released.
        """
        try:
            plt.close("all")  # Close all figures
            if os.environ.get("display_map") == "on":
                plt.close()  # Ensure the main figure is closed
        except KeyboardInterrupt:
            # Silently handle keyboard interrupts during cleanup
            pass
    def setstreet(self, xgoal, ygoal):
        for intersection in self.intersections.values():
            intersection.cost = float('inf')
            intersection.direction = None
        
        # setting goal intersection cost to 0
        curr_intersection = self.getintersection(xgoal, ygoal)
        curr_intersection.setcost(0)
        queue = [curr_intersection]
        while len(queue) != 0:
            curr = queue.pop()
            curr_cost = curr.cost
            for heading in range(8):
                if curr.streets[heading] != STATUS.CONNECTED:
                    continue
                dx, dy = DX_DY_TABLE[heading]
                neighbor_x, neighbor_y = curr.x+dx, curr.y+dy
                neighbor = self.getintersection(neighbor_x, neighbor_y)
                potential_cost = curr_cost + distance(dx, dy)
                
                if potential_cost < neighbor.cost:
                    neighbor.cost = potential_cost
                    neighbor.direction = (heading + 4) % 8
                    sortedInsert(queue, neighbor)
        return 
        
        