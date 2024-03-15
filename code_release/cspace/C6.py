import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
from q2poly import q2poly
import shapely
from shapely.geometry import Polygon as Polygon_shapely
from shapely import MultiPoint
import typing

from C1 import C1_func




def C6_func(robot: typing.Dict[str, typing.List[float]], q_path: typing.List[np.array], obstacles: typing.List[Polygon]) -> int:
    """Calculate the number of collisions that occur along the path.
    
    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters.
    q_path : typing.List[np.array]
       A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using actual angle values.
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles.

    Returns
    -------
    int
        The number of collisions that occur along the path.
    """

    ### Insert your code below: ###
    num_collisions = 0

    for i in range(1, len(q_path)):
        q1 = q_path[i]
        q_prev = q_path[i - 1]

        poly1, poly2, p11, p12 = q2poly(robot, q1)
        poly1_prev, poly2_prev, _, _ = q2poly(robot, q_prev)

        # Combine the current and previous polygons
        union1 = MultiPoint(list(Polygon_shapely(poly1).exterior.coords) + list(Polygon_shapely(poly1_prev).exterior.coords)).convex_hull
        union2 = MultiPoint(list(Polygon_shapely(poly2).exterior.coords) + list(Polygon_shapely(poly2_prev).exterior.coords)).convex_hull

        # Check for collisions with obstacles
        for obstacle in obstacles:
            obstacle_shapely = Polygon_shapely(obstacle)
            if union1.intersects(obstacle_shapely) or union2.intersects(obstacle_shapely):
                num_collisions += 1

                # Visualization of collisions (optional)
                plot_obstacles_robot(obstacles=obstacles, link1=poly1, link2=poly2, origin1=p11, origin2=p12)
                plt.xlim(0, 10)
                plt.ylim(0, 10)
                plt.xlabel("x")
                plt.ylabel("y") 
                plt.show()

    return num_collisions