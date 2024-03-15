import numpy as np
from robot import Simple_Manipulator as Robot



def M5(robot: Robot, path: np.array) -> np.array:
    """Smooth the given path

    Parameters
    ----------
    robot : Robot
        our robot object
    path : np.array
        Nx4 numpy array containing a collision-free path between q_start and q_goal

    Returns
    -------
    np.array
        Nx4 numpy array containing a smoothed version of the
        input path, where some unnecessary intermediate
        waypoints may have been removed
    """

    #student work start here



    smoothed_path = [path[0]] 

    for i in range(1, len(path)):
        if not robot.is_in_collision(path[i]):
            smoothed_path.append(path[i])

    return np.array(smoothed_path)



