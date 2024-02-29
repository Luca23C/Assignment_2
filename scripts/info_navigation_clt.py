#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import Navigation, NavigationResponse

"""
.. module:: assignment_2_2023

    :platform: Unix
    :synopsis: ROS client node that provide the average speed of the robot and its distance to the goal position 

    :moduleauthor: Luca Cornia

Client:
    /navigation_service
"""


def nav_client():
    """
    Main of the client that when called retrieve the information about the distance between the robot and the goal chosen as well as the average speed of the robot

    Parameters
    ----------

    response.distance: float
        Get the information of the distance between robot and goal
    response.average_speed: float
        Get the information of the average speed of the robot
    """
    rospy.init_node('info_nav_client')
    client = rospy.ServiceProxy('/navigation_service', Navigation)

    rospy.wait_for_service('/navigation_service')
    response = client()

    rospy.loginfo("\nThe current distance from target is: dist = %f \nThe current average speed is: average_speed = %f", response.distance, response.average_speed)


if __name__=='__main__':
    nav_client()