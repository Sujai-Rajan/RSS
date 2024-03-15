import numpy as np
from shapely.geometry import Polygon as Polygon_shapely
import typing

def q2poly(robot: typing.Dict[str, typing.List[float]], q: typing.List[float]) -> typing.Tuple[np.array, np.array, np.array, np.array]:
    """ A function that takes in the robot's parameters and a configuration and 
    returns the vertices of the robot's links after transformation and the pivot points of the links after transformation

    Parameters
    ----------
    robot : typing.dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    q : typing.List[float]
        A 2-element list representing the configuration of the robot

    Returns
    -------
    typing.Tuple[np.array, np.array, np.array, np.array]
        np.array: 
            a numpy array representing the vertices of the first link of the robot after transformation
        np.array: 
            a numpy array representing the vertices of the second link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the first link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the second link of the robot after transformation
    """


    ### Insert your code below: ###


    link1 = np.array(robot["link1"])
    link2 = np.array(robot["link2"])
    pivot1 = np.array(robot["pivot1"])


    q = np.array([-q[0], -q[1]])
    q_link2 = q[0] + q[1]


    transformation_matrix_1 = np.array([[np.cos(q[0]), -np.sin(q[0])],
                                        [np.sin(q[0]), np.cos(q[0])]])

    transformation_matrix_2 = np.array([[np.cos(q_link2), -np.sin(q_link2)],
                                        [np.sin(q_link2), np.cos(q_link2)]])
    

    shape1 = link1 @ transformation_matrix_1 + pivot1

    pivot2 = [2.1 * np.cos(-q[0]) + pivot1[0], 2.1 * np.sin(-q[0]) + pivot1[1]]
    shape2 = link2 @ transformation_matrix_2 + pivot2

    return shape1, shape2, pivot1, pivot2




    q = np.array([-q[0],-q[1]])


    link1 = np.array(robot["link1"])
    link2 = np.array(robot["link2"])
    pivot1 = np.array(robot["pivot1"])



    q_link2 = q[0]+q[1]


    transformation_matrix_1 = np.array([[np.cos(q[0]), -np.sin(q[0])],
                                       [np.sin(q[0]), np.cos(q[0])]])
    
    transformation_matrix_2 = np.array([[np.cos(q_link2), -np.sin(q_link2)],
                                       [np.sin(q_link2), np.cos(q_link2)]])
    
   
   
   
   
    shape1 = np.dot(link1, transformation_matrix_1) + pivot1
    

    pivot2 = [2.1*np.cos(-q[0]) + pivot1[0], 2.1*np.sin(-q[0]) + pivot1[1]] 

    shape2 = np.dot(link2, transformation_matrix_2) + pivot2

  

    return shape1, shape2, pivot1, pivot2
