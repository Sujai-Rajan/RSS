import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing


def C3_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array,q_grid: np.array, q_goal: np.array) -> np.array:
    """Create a new 2D array that shows the distance from each point in the configuration space to the goal configuration.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        The configuration space of the robot given by C2. The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_goal : np.array
        A 2 x 1 numpy array representing the goal configuration of the robot in the format of [q1, q2].

    Returns
    -------
    np.array
       A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration. 
       The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    """

   
 
    distances = np.zeros_like(cspace)
    goal_idx = np.argmin(np.abs(q_grid - q_goal[0]))
    goal_idy = np.argmin(np.abs(q_grid - q_goal[1]))
    distances[goal_idx, goal_idy] = 2
    distances[cspace == 1] = 1
    L = [(goal_idx, goal_idy)]

    while L:
        current_cell = L.pop(0)
        x, y = current_cell
        neighbors = [(x + dx, y + dy) for dx in range(-1, 2) for dy in range(-1, 2) if dx != 0 or dy != 0]
        for nx, ny in neighbors:
            if 0 <= nx < cspace.shape[0] and 0 <= ny < cspace.shape[1] and distances[nx, ny] == 0 and cspace[nx, ny] == 0:
                distances[nx, ny] = distances[x, y] + 1
                L.append((nx, ny))

    return distances