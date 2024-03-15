import typing
import numpy as np
from networkx import Graph
from robot import Simple_Manipulator as Robot

from M1 import M1

def M2(robot: Robot, num_samples: int, num_neighbors: int) -> typing.Tuple[np.array, Graph]:
    """ Implement the PRM algorithm

    Parameters
    ----------
    robot : Robot
        our pybullet robot class
    num_samples : int
        number of samples in PRM
    num_neighbors : int
        number of closest neighbors to consider in PRM

    Returns
    -------
    typing.Tuple[np.array, Graph]
        np.array: 
            num_samples x 4 numpy array, sampled configurations in the roadmap (vertices)
        G: 
            a NetworkX graph object with weighted edges indicating the distance between connected nodes in the joint configuration space.
            This should be impelemented as an undirected graph.
    """

    # HINTS
    # useful functions and parameters
    # robot.lower_lims, robot.upper_lims -> Joint Limits
    # robot.check_edge() -> check the linear path between 2 joint configurations for collisions
    
    ### student code start here
   
    samples = []
    while len(samples) < num_samples:
        sample = M1(robot.lower_lims, robot.upper_lims, 1)[0]
        if not robot.is_in_collision(sample):
            samples.append(sample)
    samples = np.array(samples)

   
    start_sample = np.array([0., 0., 0., 0.])
    goal_sample = np.array([0, -np.pi, 0, -np.pi])

    if not any(np.array_equal(sample, start_sample) for sample in samples):
        samples[0] = start_sample  


    if not any(np.array_equal(sample, goal_sample) for sample in samples):
        samples[-1] = goal_sample  


    G = Graph()

    for i, sample in enumerate(samples):
        G.add_node(i, configuration=sample)

  
    for i in range(num_samples):
        for j in range(i + 1, num_samples):
            if robot.check_edge(samples[i], samples[j], resolution=10):
                distance = np.linalg.norm(samples[i] - samples[j])
                G.add_edge(i, j, weight=distance)

    

    return samples, G