#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import Position, PositionResponse

"""
.. module:: assignment_2_2023

    :platform: Unix
    :synopsis: ROS client node that provide the last goal position setted

    :moduleauthor: Luca Cornia

Client:
    /last_pose
"""


def position_client():
    """
    Main of the client that when is executed call the service which retrieve the last goal selected

    Parameters
    ----------

    response.x_res: float
        Contain the information about the x coordinate of the last goal selected
    response.y_res: float
        Contain the information about the y coordinate of the last goal selected
    """
    
    rospy.init_node('last_target_position_client')
    client = rospy.ServiceProxy('/last_pose', Position)

    rospy.wait_for_service('/last_pose')
    response = client()

    rospy.loginfo("Last target coordinate are: x: %f, y: %f", response.x_res, response.y_res)


if __name__=='__main__':
    position_client()