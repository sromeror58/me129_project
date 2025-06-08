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
            # print(f'All paths exhausted, going back to manual mode.')
            print(f'All paths exhausted, Clearing blockages and re-attempting.')
            map.clear_blocked()
            # # set commands and stuff, do later
            # if shared.acquire():
            #     try:
            #         shared.mode = 0
            #         shared.goal = None
            #     finally:
            #         shared.release()
            return True, None
    return False, None

def fetch(map, pose, shared, take_step):
    """
    Determines the next move in a directed exploration to fetch a prize.

    Parameters:
        map (Map): The robot's map of the environment.
        pose (Pose): The robot's current position and heading.
        shared (SharedData): Shared state object.
        take_step (function): Function to take a step in the environment.

    Returns:
        tuple: (bool indicating whether to continue exploring, 
                str or None for the next command).
    """
    current_intersection = map.getintersection(pose.x, pose.y)
    command = None
    # checking if prize exist or if prize is in the dictionary
    if shared.acquire():
        try:
            prize = shared.fetch
            if prize is None:
                print(f'Prize to fetch is None, going back to manual mode')
                shared.mode = 0
                shared.fetch = None
        finally:
            shared.release()
        if prize is None:
            return False, None, None
    distance_dict = current_intersection.distance_dict
    if prize not in distance_dict:
        if shared.acquire():
            try:
                print(f'Prize to fetch does not exist, going back to manual mode')
                shared.mode = 0
                shared.fetch = None
            finally:
                shared.release()
            # print('IM THE ERROR BBBB!!!!')
            return False, None, None

    num_intersections = {}
    for intersection in map.intersections.values():
        nfc_id = intersection.nfc_id
        # print((intersection.x, intersection.y))
        # print(intersection.distance_dict)
        dist_to_prize = intersection.distance_dict[prize]['distance']
        if dist_to_prize == 0.5:
            num_intersections[(intersection.x, intersection.y)] = nfc_id
    # case 1: see if there exist two intersections that are 0.5 away from the prize
    print(f'Current intersections with distance of 0.5: {num_intersections}')
    if len(num_intersections) == 2:
        print(f'2 INTERSECTIONS WITH DISTANCE OF 0.5')
        # this is the case when we are currently at an intersection of distance 0.5 
        if (pose.x, pose.y) in num_intersections.keys():
            return False, None, num_intersections
        else:
            goal = next(iter(num_intersections))
            continue_exploring, command = go_to_goal(take_step, goal, shared, map, pose)
            # print('IM THE ERROR CCCC!!!!')
            return continue_exploring, command, num_intersections
    # case 2: only one intersection exist, so explore until we find the other, which is only up to 3 different
    # candidates
    elif len(num_intersections) == 1:
        print(f'1 INTERSECTIONS WITH DISTANCE OF 0.5')
        goal = next(iter(num_intersections))
        # if we are at an intersection with distance 0.5, we check all headings and explore them to get the other intersection.
        if (pose.x, pose.y) == goal:
            if any(status == STATUS.UNKNOWN for status in current_intersection.streets):
                command = 'left'
                return True, command, num_intersections
            else:
                potential_candidates = []
                for h in range(8):
                    if current_intersection.streets[h] in [STATUS.UNEXPLORED] and not current_intersection.blocked[h]:
                        dx, dy = DX_DY_TABLE[h]
                        next_x, next_y = current_intersection.x + dx, current_intersection.y + dy
                        potential_candidates.append((next_x, next_y))
                if len(potential_candidates) == 0:
                    print(f'No way to get to goal from here, clearing blockages and re-attempting.')
                    map.clear_blocked()
                    return True, None, num_intersections
                continue_exploring, command = directed_explore(map, pose, potential_candidates[0], command, shared)
                return True, command, num_intersections
        # otherwise, we explored a intersection that had a distance that increased, so we go back to original intersection.
        else:
            goal = next(iter(num_intersections))
            continue_exploring, command = go_to_goal(take_step, goal, shared, map, pose)
            return True, command, num_intersections
    # case 3: We have not explored an intersection that has a distance of 0.5
    else:
        print(f'0 INTERSECTIONS WITH DISTANCE OF 0.5')
        # first check if we can go to the intersection that is closest to the prize that is not the current intersection
        lowest_intersection = current_intersection
        for intersection in map.intersections.values():
            if intersection.distance_dict[prize]['distance'] < lowest_intersection.distance_dict[prize]['distance']:
                lowest_intersection = intersection
        # we found an intersection that is closest to the prize that is not the current intersection
        if (pose.x, pose.y) != (lowest_intersection.x, lowest_intersection.y):
            goal = (lowest_intersection.x, lowest_intersection.y)
            continue_exploring, command = go_to_goal(take_step, goal, shared, map, pose)
            return True, command, num_intersections
        # we are at an intersection with the lowest distance to the prize, so we explore
        else:
            # get all unexplored streets if they have not been discovered
            if any(status == STATUS.UNKNOWN for status in lowest_intersection.streets):
                command = 'left'
                return True, command, num_intersections
            # otherwise, consider all unexplored streets and explore to new intersection
            else:
                potential_candidates = []
                for h in range(8):
                    if lowest_intersection.streets[h] in [STATUS.UNEXPLORED] and not lowest_intersection.blocked[h]:
                        dx, dy = DX_DY_TABLE[h]
                        next_x, next_y = lowest_intersection.x + dx, lowest_intersection.y + dy
                        potential_candidates.append((next_x, next_y))
                print(f'Potential Candidates: {potential_candidates}')
                continue_exploring, command = directed_explore(map, pose, potential_candidates[0], command, shared)
                return True, command, num_intersections

def go_to_goal(take_step, goal, shared, map, pose):
    """
    Takes a step towards a goal.

    Parameters:
        take_step (bool): Should the robot take a step?
        goal (tuple): The coordinates of the goal.
        shared (SharedData): Shared state object.
        map (Map): The robot's map of the environment.
        pose (Pose): The robot's current position and heading.

    Returns:
        tuple: A tuple indicating whether to continue exploring and the next command.
    """
    taking_step = take_step
    # If we have a goal, check if we've reached it
    if goal is not None:
        if take_step:
            if shared.acquire():
                try:
                    shared.take_step = False
                finally:
                    shared.release()
        # Check if goal exists in the map to determine if this is directed exploration
        is_directed_exploration = goal not in map.intersections
        
        if (pose.x, pose.y) == goal:

            # If was in goal mode, go to manual
            # If was in autonomous mode, clear goal
            if shared.acquire():
                try:
                    if shared.mode == 2:
                        shared.mode = 0
                        print("Goal reached! Returning to manual mode from goal mode")
                    elif shared.mode == 1:
                        print("Goal reached! Continuing autonomous mode")
                    elif shared.mode == 3 and shared.before_pause == 2:
                        print("Goal reached after the final step! Returning to manual mode from goal mode")
                        shared.mode = 0
                    shared.goal = None
                finally:
                    shared.release()
            map.setstreet(None, None)  # Clear optimal path tree
            map.plot(pose)
            return False, None
        if not is_directed_exploration:
            map.setstreet(goal[0], goal[1])
            # We haven't reached goal, so get current intersection and check optimal direction
            current = map.getintersection(pose.x, pose.y)
            optimal_heading = current.direction
            if optimal_heading is not None:
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
                # there is no path to get to intersection, so we explore any unexplored streets to get to goal
                is_directed_exploration = True
                print("No valid known path to goal. Switching to directed explore to get to goal.")
                map.plot(pose)
                return True, None
                # continue
        else:
            # Directed exploration: Goal doesn't exist in map yet
            # Use exploration logic but biased toward the goal
            is_directed_exploration, optimal_command = directed_explore(map, pose, goal, None, shared)
            if is_directed_exploration:
                command = optimal_command
            return True, command