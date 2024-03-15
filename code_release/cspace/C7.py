import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *

def C7_func(cspace: np.array) -> np.array:
    """Pad the configuration space by one grid cell.

    Parameters
    ----------
    cspace : np.array
        The origianl configuration space of the robot.

    Returns
    -------
    np.array
        The padded configuration space of the robot.
    """

    
    padding_value = 1

    
    padded_cspace = np.copy(cspace)

    
    rows, cols = cspace.shape

   
    for i in range(rows):
        for j in range(cols):
         
            if cspace[i, j] == 1:
                
                if i > 0:
                    padded_cspace[i - 1, j] = padding_value
                if i < rows - 1:
                    padded_cspace[i + 1, j] = padding_value
                if j > 0:
                    padded_cspace[i, j - 1] = padding_value
                if j < cols - 1:
                    padded_cspace[i, j + 1] = padding_value
                if i > 0 and j > 0:
                    padded_cspace[i - 1, j - 1] = padding_value
                if i > 0 and j < cols - 1:
                    padded_cspace[i - 1, j + 1] = padding_value
                if i < rows - 1 and j > 0:
                    padded_cspace[i + 1, j - 1] = padding_value
                if i < rows - 1 and j < cols - 1:
                    padded_cspace[i + 1, j + 1] = padding_value

    return padded_cspace