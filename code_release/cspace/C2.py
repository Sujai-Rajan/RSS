import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as Polygon_shapely
from helper_functions import *
from q2poly import q2poly
import typing





def C2_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array, obstacles: typing.List[Polygon_shapely], q_grid: np.array) -> np.array:
    """Create the configuration space for the robot with the given obstacles in the given empty cspace array.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        An empty 2D numpy array
    obstacles : typing.List[Polygon_shapely]
        A list of Shapely polygons representing the obstacles
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.

    Returns
    -------
    np.array
        A 2D numpy array representing the updated configuration space.
    """
 
    cspace_resolution = cspace.shape[0]

    
    obstacles_shapely = [Polygon_shapely(polygon) for polygon in obstacles]

    
    for i in range(cspace_resolution):
        for j in range(cspace_resolution):
            
            q = [q_grid[i], q_grid[j]]

           
            transformed_link1, transformed_link2, _, _ = q2poly(robot, q)

           
            poly_link1 = Polygon_shapely(transformed_link1)
            poly_link2 = Polygon_shapely(transformed_link2)

        
            collision = any(poly_link1.intersects(obstacle) or poly_link2.intersects(obstacle) for obstacle in obstacles_shapely)

           
            cspace[i, j] = collision

    return cspace
