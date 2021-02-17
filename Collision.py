import math
import enum
from enum import IntEnum

'''
All the computations for the collision class will be done for the future.
The task is to predict whether the movement in the next time frame in the 
given direction will cause any collisions or not. 

If any collision are detected in this module, we will clip/disable the directional
velocity of our robot so as to only enable parallel motion to the walls.
'''

class Walls(IntEnum):
    bottom = 0
    right = 1
    top = 2
    left = 3

def min_distance_point_to_line(A, B, C):
    '''Returns the shortest distance from a point to a line segment'''
    x1 = B[0] - A[0]
    y1 = B[1] - A[1]
    x2 = C[0] - A[0]
    y2 = C[1] - A[1]

    divisor = math.sqrt(x1 * x1 + y1 * y1)
    return abs(x1 * y2 - y1 * x2)/divisor

def calculate_dist_from_walls(boundary_points, C, radius):
    '''Args: 
        boundary_points - list of 4 (x, y) coordinates,
        C - (x,y) for the center of circle,
        radius - radius of the circle

        Return - list of distance of center from each wall.
    '''
    distance_to_walls = [None]*4

    distance_to_walls[Walls.bottom] = min_distance_point_to_line(boundary_points[0], boundary_points[1], C) - radius
    distance_to_walls[Walls.right] = min_distance_point_to_line(boundary_points[1], boundary_points[2], C) - radius
    distance_to_walls[Walls.top] = min_distance_point_to_line(boundary_points[2], boundary_points[3], C) - radius
    distance_to_walls[Walls.left] = min_distance_point_to_line(boundary_points[3], boundary_points[0], C) - radius
    return distance_to_walls
    
def check_for_collision(boundary_points, C, radius):
    '''Args:

    Return:

    This function will detect which wall or walls can the collision occur on in the next movement.
    Further, in the velocity controller module, we will need to know the walls are involved and the
    robot's current angle so as to be able to control the speed.
    '''

    collision_verdict_list = []
    distance_to_walls = calculate_dist_from_walls(boundary_points, C, radius)

    # Object passes through the wall/s - Move is not allowed
    for ind in range(len(distance_to_walls)):
        if(distance_to_walls[ind] < 0):
            return collision_verdict_list
    
    # Just touches the walls or Away from the walls - Move is allowed
    for wall in Walls:
        if(distance_to_walls[wall] >= 0):
            collision_verdict_list.append(wall)
    return collision_verdict_list

if __name__ == "__main__":
    P1 = [0, 0]
    P2 = [6, 0]
    P3 = [6,6]
    P4 = [0,6]
    C = [0.5, 1]

    boundary_points = [P1, P2, P3, P4]
    print('\n')
    print(check_for_collision(boundary_points, C, 1))