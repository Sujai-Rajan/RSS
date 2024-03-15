from networkx import Graph, shortest_path
import numpy as np
from robot import Simple_Manipulator as Robot
import typing

def M3(robot: Robot, samples: np.array, G: Graph, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """ Find a path from q_start to q_goal using the PRM roadmap

    Parameters
    ----------
    robot : Robot
        our robot object
    samples : np.array
        num_samples x 4 numpy array of nodes/vertices in the roadmap
    G : Graph
        An undirected NetworkX graph object with the number of nodes equal to num_samples, 
        and weighted edges indicating collision-free connections in the robot's configuration space
    q_start : np.array
        1x4 numpy array denoting the start configuration
    q_goal : np.array
        1x4 numpy array denoting the goal configuration

    Returns
    -------
    typing.Tuple[np.array, bool]
        np.array:
            Nx4 numpy array containing a collision-free path between
            q_start and q_goal, if a path is found. The first row
            should be q_start, the final row should be q_goal.
        bool:
            Boolean denoting whether a path was found
    """

 
    start_node_distances = np.linalg.norm(samples - q_start, axis=1)
    goal_node_distances = np.linalg.norm(samples - q_goal, axis=1)

    sort_start_distance = np.argsort(start_node_distances)
    sort_goal_distance = np.argsort(goal_node_distances)

    start_neighbours = samples[sort_start_distance]
    goal_neighbours = samples[sort_goal_distance]

    on_start = None
    off_goal = None

 
    for i in range(len(start_neighbours)):
        collision = robot.is_in_collision(start_neighbours[i])
        if not collision:
            on_start = sort_start_distance[i]
            break

 
    for i in range(len(goal_neighbours)):
        collision = robot.is_in_collision(goal_neighbours[i])
        if not collision:
            off_goal = sort_goal_distance[i]
            break


    if on_start is not None and off_goal is not None:
        path_indices = shortest_path(G, source=on_start, target=off_goal)
        path = samples[path_indices]
        path = np.vstack((q_start, path, q_goal))
        path_found = len(path_indices) > 0
    else:
        path = np.array([])
        path_found = False
        

    return path, path_found