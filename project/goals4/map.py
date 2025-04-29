from enum import Enum

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

class Map:
    def __init__(self):
        self.intersections = {}

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

    def outcomeA(self, x, y, heading1, heading2, isLeft: bool):
        """
        Set streets to STATUS.NONEXISTENT based on the direction of turn from heading1 to heading2.
        
        Args:
            x (float): Longitude coordinate of the intersection
            y (float): Latitude coordinate of the intersection
            heading1 (int): Starting heading (0-7)
            heading2 (int): Ending heading (0-7)
            isLeft (bool): True if turning left, False if turning right
        """
        intersection = self.getintersection(x, y)
        
        if isLeft:
            # Left turn
            if heading1 < heading2:
                for h in range(heading1 + 1, heading2):
                    intersection.streets[h] = STATUS.NONEXISTENT
            else:  
                for h in range(heading1 + 1, 8):
                    intersection.streets[h] = STATUS.NONEXISTENT
                for h in range(0, heading2):
                    intersection.streets[h] = STATUS.NONEXISTENT
        else:  
            # Right turn
            if heading2 < heading1:
                for h in range(heading2 + 1, heading1):
                    intersection.streets[h] = STATUS.NONEXISTENT
            else:  
                for h in range(heading2 + 1, 8):
                    intersection.streets[h] = STATUS.NONEXISTENT
                for h in range(0, heading1):
                    intersection.streets[h] = STATUS.NONEXISTENT

    def outcomeC(self, h0, x0, y0, x1, y1):
        intersection = self.getintersection(x0, y0)
        intersection.streets[h0] = STATUS.DEADEND

        intersection = self.getintersection(x1, y1)
        intersection.streets[(h0 + 4) % 8] = STATUS.CONNECTED
        for i in range(0, 8):
            if i != h0:
                intersection.streets[i] = STATUS.NONEXISTENT

        

        