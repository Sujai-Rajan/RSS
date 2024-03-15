from M1 import M1
import numpy as np
from robot import Simple_Manipulator as Robot
import typing

def M4(robot: Robot, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """Implement RRT algorithm to find a path from q_start to q_goal

    Parameters
    ----------
    robot : Robot
        our robot object
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

    
    # Hyperparameters
    step_size = 0.25  
    goal_sample_frequency = 0.35
    max_iterations = 1000


    #Initialising  Tree 
    tree = [(q_start, -1)]  

    # Main RRT loop
    for _ in range(max_iterations):  
        if np.random.rand() < goal_sample_frequency:
            q_rand = q_goal  
        else:
            q_rand = M1(robot.lower_lims, robot.upper_lims, 1)[0]

      
        nearest_node_idx = np.argmin(np.linalg.norm(np.array([node[0] for node in tree]) - q_rand, axis=1))
        q_near, parent_idx = tree[nearest_node_idx]

        
        q_new = q_near + step_size * (q_rand - q_near) / np.linalg.norm(q_rand - q_near)


        if not robot.check_edge(q_near, q_new):
            continue  

        tree.append((q_new, nearest_node_idx))

       
        if np.linalg.norm(q_new - q_goal) < step_size:
           
            path = [q_goal]
            current_idx = len(tree) - 1

            while current_idx != 0:
                path.append(tree[current_idx][0])
                current_idx = tree[current_idx][1]

            path.reverse()
            path.insert(0, q_start)
            
            return np.array(path), True

    return np.array([]), False  




