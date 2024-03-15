import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing

def C4_func(distances: np.array,q_grid: np.array, q_start: np.array) -> typing.List[np.array]:
    """Using the distance array from C3, find the optimal path from the start configuration to the goal configuration (zero value).

    Parameters
    ----------
    distances : np.array
        A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration.
        This is given by C3 
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_start : np.array
        A 2 x 1 numpy array representing the start configuration of the robot in the format of [q1, q2].

    Returns
    -------
    typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using indices of q_grid.
        Example: [ [q1_0 , q2_0], [q1_1, q2_1], .... ]
    """
    
    ### Insert your code below: ###
    rows, cols = distances.shape
    start_x = np.argmin(np.abs(q_grid - q_start[0]))
    start_y = np.argmin(np.abs(q_grid - q_start[1]))

    checked = np.zeros((rows, cols))
    q_goal = np.argwhere(distances == 2)[0]

    if not q_goal.size:
        return []

    neighbors = []
    current = np.array([start_x, start_y])
    path = [current]

    while not np.array_equal(current, q_goal):
        neighbors = []
        x, y = current

        for i in range(-1, 2):
            for j in range(-1, 2):
                nx, ny = x + i, y + j
                if 0 <= nx < rows and 0 <= ny < cols and distances[nx, ny] != 1 and checked[nx, ny] != 1:
                    neighbors.append([nx, ny])

        if not neighbors:
            break

        min_dist = np.inf
        next_idx = None

        for neighbor in neighbors:
            nx, ny = neighbor
            if distances[nx, ny] < min_dist:
                min_dist = distances[nx, ny]
                next_idx = neighbor

        current = next_idx
        path.append(current)
        checked[current[0], current[1]] = 1

    return path
