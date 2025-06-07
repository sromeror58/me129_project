# Imports
from behaviors import Behaviors
from pose import Pose
from map import Map, Intersection, STATUS, distance, sortedInsert
import time
import math
from config import DX_DY_TABLE, EUC_CONST
import copy

def euclidean_distance(x_i, y_i, x_g, y_g):
    """
    Calculates the Euclidean distance between two points.

    Parameters:
        x_i (int): x-coordinate of the initial point.
        y_i (int): y-coordinate of the initial point.
        x_g (int): x-coordinate of the goal point.
        y_g (int): y-coordinate of the goal point.

    Returns:
        float: Scaled Euclidean distance between the points.
    """
    return EUC_CONST * math.sqrt((x_g - x_i)**2 + (y_g - y_i)**2)

def directed_explore(map, pose, goal, command, shared):
    """
    Determines the next move in a directed exploration toward a goal.

    Parameters:
        map (Map): The robot's map of the environment.
        pose (Pose): The robot's current position and heading.
        goal (tuple): Target coordinates as (x, y).
        command (str or None): Previous movement command.
        shared (SharedData): Shared state object.

    Returns:
        tuple: (bool indicating whether to continue exploring, 
                str or None for the next command).
    """
    command = command.copy() if command is not None else command
    x_i, y_i = pose.x, pose.y
    x_g, y_g = goal[0], goal[1]
    # check if this current position is in the map (it should always exist)
    # if (x_i, y_i) not in map.intersections:
    #     print(f'Position {(curr_x, curr_y)} not in current map. Stopping!')
    #     return False
    # check if we are already at our goal
    if (x_i, y_i) == (x_g, y_g):
        if shared.acquire():
            try:
                shared.mode = 0
                print(f'Goal Reached! Finished.')
                shared.goal = None
            finally:
                shared.release()
        map.setstreet(None, None)
        return False, None
    # first, we want to turn around at our current intersection, to get 
    # the most information as possible (i.e. get all possible headings)
    curr_intersection = map.getintersection(x_i, y_i)
    if any(status == STATUS.UNKNOWN for status in curr_intersection.streets):
        command = 'left'
        return True, command
    if any(status == STATUS.UNEXPLORED for intersection in map.intersections.values() for status in intersection.streets):
    # if any(status == STATUS.UNEXPLORED for status in curr_intersection.streets):
        queue = [] # each element is (cost to goal, Intersection, direction)
        # checking all other intersections that have unexplored headings and not
        # blocked such that we only add an intersection once (no duplicate intersections)
        # in queue
        for intersection in map.intersections.values():
            min_h, min_dist = 0, float("inf")
            for h in range(8):
                if intersection.streets[h] in [STATUS.UNEXPLORED, STATUS.CONNECTED] and not intersection.blocked[h]:
                    dx, dy = DX_DY_TABLE[h]
                    next_x, next_y = intersection.x + dx, intersection.y + dy
                    euc_distance = euclidean_distance(next_x, next_y, x_g, y_g) + distance(dx, dy) # since we are travelling 1 block
                    if euc_distance < min_dist:
                        min_h, min_dist = h, euc_distance
            if min_dist < float("inf"):
                queue.append((min_dist, intersection, min_h))
        
        # we call our modified dijkstra's on this queue of potential candidates that would
        # get us to the goal
        map.directed_explore_dijkstras(queue)
        optimal_heading = map.getintersection(x_i, y_i).direction
        # If current heading doesn't match optimal direction, need to turn
        if pose.heading != optimal_heading:
            current_heading = pose.heading

            # Calculate the difference in heading turning left
            heading_diff = (optimal_heading - current_heading) % 8

            if heading_diff <= 4:
                command = 'left'  # Turn left
            else:
                command = 'right'  # Turn right
        else:
            command = 'straight' # going straight along optimal path
        return True, command
    else:
        # check if all paths are exhausted which we go back to manual mode
        if map.find_nearest_unexplored(x_i, y_i) is None:
            print(f'All paths exhausted, going back to manual mode.')
            # set commands and stuff, do later
            if shared.acquire():
                try:
                    shared.mode = 0
                    shared.goal = None
                finally:
                    shared.release()
            return False, None
    return False, None
    

def fetch(map, pose, command, shared, prev, curr):
    prize = None
    nfc_id = None
    prize_dict = None
    info_dict = None
    # get the prize where if it is None, return and quit fetching
    if shared.acquire():
        try:
            prize = shared.fetch
            nfc_id = shared.intersection_id
            prize_dict = shared.prize_dict
            info_dict = shared.info_dict
            if prize is None:
                print(f'Prize to fetch is None, going back to manual mode')
                shared.mode = 0
                shared.fetch = None
        finally:
            shared.release()
    # updating whether our previous intersection and current
    if nfc_id != curr:
        prev = curr
        curr = nfc_id
    
    # check the valid cases of prev and curr (i.e. when we have no prev and when we have a prev intersection)
    if prev is None:
        # first, we want to turn around at our current intersection, to get 
        # the most information as possible (i.e. get all possible headings)
        x_i, y_i = pose.x, pose.y  
        curr_intersection = map.getintersection(x_i, y_i)
        if any(status == STATUS.UNKNOWN for status in curr_intersection.streets):
            command = 'left'
            return prize, command, prev, curr
        # else we go straight or turn to the next intersection
        else:
            candidates = map.get_potential_streets(pose.x, pose.y)
            closest_heading = min(candidates, key=lambda x: min((x[0] - pose.heading) % 8, (pose.heading - x[0]) % 8))[0] if candidates else None
            if closest_heading is not None:
                if closest_heading != pose.heading:
                    heading_diff = (closest_heading - pose.heading) % 8
                    if heading_diff <= 4:
                        command = 'left'  # Turn left
                    else:
                        command = 'right'  # Turn right
                else:
                    command = 'straight' # going straight along optimal path
            # this else case should never happen!!!
            else:
                print('There is no more streets to explore to fetch, going back to manual mode!')
                if shared.acquire():
                    shared.mode = 0
                    shared.fetch = None
                    shared.release()
            return prize, command, prev, curr
    # checking when prev is not None
    else:
        # first we want to compare the cost of the previous nfc id to the current nfc id
        prev_distance = dist_dict[prev][prize]['distance']
        curr_distance = dist_dict[curr][prize]['distance']
        # case when fetch is complete
        if curr_distance == 0.5 and prev_distance == 0.5:
            print(f'NFC ids surrounding prize are: ({prev, curr})')
            if shared.acquire():
                    shared.mode = 0
                    shared.fetch = None
                    shared.release()
            return prize, None, prev, curr   
        # case when our previous intersection was closer to the prize
        elif prev_distance < curr_distance:
            # set previous to None and build off of the current intersection
            prev = None
            
            pass
        # case when our current intersection is better if not the same
        else:
            pass
            