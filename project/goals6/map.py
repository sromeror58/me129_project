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
    """
    Returns the movement cost based on direction.

    Diagonal steps cost 1.4, and straight (N, S, E, W) steps cost 1.

    Args:
            dx (int): Change in x-direction
            dy (int): Change in y-direction
    """
    if (dx, dy) in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
        return 1
    return 1.4


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

    UNKNOWN = 0 # black
    NONEXISTENT = 1 # grey
    UNEXPLORED = 2 # blue
    DEADEND = 3 # red
    CONNECTED = 4 # green


class DIJKSTRA_STATE(Enum):
    UNVISITED = 0  # Blue (already UNKNOWN)
    ONDECK = 1  # Green
    PROCESSED = 2  # Brown


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
        self.cost = float("inf")
        self.direction = None
        self.dijkstra_state = DIJKSTRA_STATE.UNVISITED
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

    def four_roads_rule(self, pose):
        """
        We know that at each intersection, roads are necessarily separated by at least 90 degrees.
        This means that if there are 3 roads so far at an intersection, if consecutive have >90 deg sep, then rest of roads do not exist.
        """
        intersection = self.getintersection(pose.x, pose.y)
        
        # Get all existing roads at this intersection
        existing_roads = []
        for heading in range(8):
            if intersection.streets[heading] not in [STATUS.UNKNOWN, STATUS.NONEXISTENT]:
                existing_roads.append(heading)
        
        # If we have 3 roads, check if any consecutive pair has >90 degree separation
        if len(existing_roads) == 3:
            # Sort roads by heading
            existing_roads.sort()
            
            # Check each consecutive pair
            for i in range(3):
                road1 = existing_roads[i]
                road2 = existing_roads[(i + 1) % 3]
                
                # Calculate minimum separation between roads
                separation = min(abs(road2 - road1), 8 - abs(road2 - road1))
                
                # If separation > 2 (90 degrees), mark remaining directions as non-existent
                if separation == 3:
                    # Mark all other directions as non-existent
                    for heading in range(8):
                        if heading not in existing_roads:
                            intersection.updateStreet(heading, STATUS.NONEXISTENT)

                    print("FOUR ROADS RULE")

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
        intersection.updateStreet((h1 + 1) % 8, STATUS.NONEXISTENT)
        intersection.updateStreet((h1 - 1) % 8, STATUS.NONEXISTENT)
        
        self.four_roads_rule(pose1)

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
            # checking if there is a road ahead and if there is a unexplored or connected
            # road within +- 45 degrees then we keep it as unexplored
            intersection.updateStreet((pose1.heading + 1) % 8, STATUS.NONEXISTENT)
            intersection.updateStreet((pose1.heading - 1) % 8, STATUS.NONEXISTENT)
        else:
            intersection.updateStreet(pose1.heading, STATUS.NONEXISTENT)
        # intersection.streets[pose1.heading] = STATUS.UNEXPLORED if road_ahead else STATUS.NONEXISTENT

        self.four_roads_rule(pose1)

        self.plot(pose1)

    def outcomeC(self, pose0, pose1, road_ahead):
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

        intersection = self.getintersection(pose1.x, pose1.y)
        if not road_ahead:
            intersection.streets[pose1.heading] = STATUS.NONEXISTENT
        if road_ahead:
            intersection.updateStreet(pose1.heading, STATUS.UNEXPLORED)

        self.four_roads_rule(pose1)

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

    def plot_no_robot(self):
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

        # Define colors for each status
        dijkstra_colors = {
            DIJKSTRA_STATE.UNVISITED: "blue",
            DIJKSTRA_STATE.PROCESSED: "saddlebrown",
            DIJKSTRA_STATE.ONDECK: "green",
        }

        for (x_int, y_int), intersection in self.intersections.items():
            color = dijkstra_colors.get(intersection.dijkstra_state)
            plt.plot(x_int, y_int, color=color, marker="o", markersize=10)
            optimal_heading = intersection.direction
            if optimal_heading is not None:
                # Get the direction vector for this heading
                dx, dy = DX_DY_TABLE[optimal_heading]
                dx *= 0.5
                dy *= 0.5
                # Draw line from intersection halfway to next
                plt.plot([x_int, x_int + dx], [y_int, y_int + dy], color="green")

        # If using TkAgg backend, display the plot
        if os.environ.get("display_map") == "on":
            try:
                plt.draw()
                plt.pause(0.1)  # Small pause to allow the plot to update
            except KeyboardInterrupt:
                # Silently handle keyboard interrupts during plotting
                pass

    def save_map(self, filename=""):
        """
        Create the plot and save it to a file instead of displaying it.

        This method is useful for saving map visualizations to files for later review
        or for generating reports.
        """
        # self.plot(pose)

        # Instead of plt.pause(), save the figure to a file
        if not filename:
            filename = "mymap"
        filename = f"plots/{filename}.pickle"
        print("Saving the map to %s..." % filename)
        with open(filename, "wb") as file:
            pickle.dump(self, file)
        # plt.savefig(filename)
        print(f"Saved map to {filename}")

    def load_map(self, filename="mymap"):
        # filename = f"plots/map_x{pose.x}_y{pose.y}_h{pose.heading}.pickle"
        filename = f"plots/{filename}.pickle"
        print("Loading the map from %s..." % filename)
        with open(filename, "rb") as file:
            map = pickle.load(file)
        print("Loaded map")
        return map

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
        """
        Finds the shortest path from every intersection to the goal at (xgoal, ygoal) (Dijkstra's).

        Args:
            xgoal (int): X-coordinate of the goal intersection.
            ygoal (int): Y-coordinate of the goal intersection.
        """
        # Reset all intersections
        for intersection in self.intersections.values():
            intersection.cost = float("inf")
            intersection.direction = None
            intersection.dijkstra_state = DIJKSTRA_STATE.UNVISITED

        # If coordinates are None, just reset the states and return
        if xgoal is None or ygoal is None:
            return

        # Check if goal intersection exists
        if (xgoal, ygoal) not in self.intersections:
            print(f"Warning: Goal intersection ({xgoal}, {ygoal}) does not exist in the map")
            return

        # setting goal intersection cost to 0
        curr_intersection = self.getintersection(xgoal, ygoal)
        curr_intersection.setcost(0)
        queue = [curr_intersection]
        
        while queue:
            curr = queue.pop(0)  # Get the lowest cost node
            if curr.dijkstra_state == DIJKSTRA_STATE.PROCESSED:
                continue  # Skip if already processed
                
            curr.dijkstra_state = DIJKSTRA_STATE.PROCESSED
            curr_cost = curr.cost
            
            for heading in range(8):
                if curr.streets[heading] != STATUS.CONNECTED:
                    continue
                    
                dx, dy = DX_DY_TABLE[heading]
                neighbor_x, neighbor_y = curr.x + dx, curr.y + dy
                
                # Check if neighbor exists in the map
                if (neighbor_x, neighbor_y) not in self.intersections:
                    continue
                    
                neighbor = self.getintersection(neighbor_x, neighbor_y)
                potential_cost = curr_cost + distance(dx, dy)

                if potential_cost < neighbor.cost:
                    neighbor.cost = potential_cost
                    neighbor.direction = (heading + 4) % 8
                    if neighbor.dijkstra_state != DIJKSTRA_STATE.ONDECK:
                        neighbor.dijkstra_state = DIJKSTRA_STATE.ONDECK
                        sortedInsert(queue, neighbor)
                        
        print("Dijkstra's Complete")
        return

    def has_unexplored_streets(self, x, y):
        """
        Check if the intersection at (x,y) has any unexplored or unknown streets.
        
        Args:
            x (int): X-coordinate of the intersection
            y (int): Y-coordinate of the intersection
            
        Returns:
            bool: True if there are unexplored streets, False otherwise
        """
        intersection = self.getintersection(x, y)
        return any(status in [STATUS.UNKNOWN, STATUS.UNEXPLORED] for status in intersection.streets)

    def get_unexplored_streets(self, x, y):
        """
        Get a list of headings with unexplored or unknown streets at the intersection.
        
        Args:
            x (int): X-coordinate of the intersection
            y (int): Y-coordinate of the intersection
            
        Returns:
            tuple: (street heading, street status) for each unexplored street
        """
        intersection = self.getintersection(x, y)
        return [(h, intersection.streets[h]) for h in range(8) if intersection.streets[h] in [STATUS.UNKNOWN, STATUS.UNEXPLORED]]

    def find_nearest_unexplored(self, start_x, start_y):
        """
        Find the nearest intersection with unexplored streets using Dijkstra's algorithm.
        Excludes the current intersection (start_x, start_y) from the search.
        
        Args:
            start_x (int): Starting X-coordinate
            start_y (int): Starting Y-coordinate
            
        Returns:
            tuple: (x, y) coordinates of nearest unexplored intersection, or None if none found
        """
        # Reset all intersections
        for intersection in self.intersections.values():
            intersection.cost = float("inf")
            intersection.direction = None
            intersection.dijkstra_state = DIJKSTRA_STATE.UNVISITED

        # Check if start intersection exists
        if (start_x, start_y) not in self.intersections:
            print(f"Warning: Start intersection ({start_x}, {start_y}) does not exist in the map")
            return None

        # Set start intersection cost to 0
        start = self.getintersection(start_x, start_y)
        start.setcost(0)
        queue = [start]
        
        while queue:
            curr = queue.pop()  # Get the lowest cost node (since queue is sorted)
            if curr.dijkstra_state == DIJKSTRA_STATE.PROCESSED:
                continue  # Skip if already processed
                
            curr.dijkstra_state = DIJKSTRA_STATE.PROCESSED
            
            # If this intersection has unexplored streets and it's not the start intersection
            if (curr.x != start_x or curr.y != start_y) and self.has_unexplored_streets(curr.x, curr.y):
                return curr.x, curr.y
                
            # Explore neighbors
            for heading in range(8):
                if curr.streets[heading] != STATUS.CONNECTED:
                    continue
                    
                dx, dy = DX_DY_TABLE[heading]
                neighbor_x, neighbor_y = curr.x + dx, curr.y + dy
                
                # Check if neighbor exists in the map
                if (neighbor_x, neighbor_y) not in self.intersections:
                    continue
                    
                neighbor = self.getintersection(neighbor_x, neighbor_y)
                potential_cost = curr.cost + distance(dx, dy)
                
                if potential_cost < neighbor.cost:
                    neighbor.cost = potential_cost
                    neighbor.direction = (heading + 4) % 8
                    if neighbor.dijkstra_state != DIJKSTRA_STATE.ONDECK:
                        neighbor.dijkstra_state = DIJKSTRA_STATE.ONDECK
                        sortedInsert(queue, neighbor)
        
        return None  # No unexplored intersections found
