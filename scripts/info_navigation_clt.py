#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import Navigation, NavigationResponse


def nav_client():

    rospy.init_node('info_nav_client')
    client = rospy.ServiceProxy('/navigation_service', Navigation)

    rospy.wait_for_service('/navigation_service')
    response = client()

    rospy.loginfo("\nThe current distance from target is: dist = %f \nThe current average speed is: average_speed = %f", response.distance, response.average_speed)


if __name__=='__main__':
    nav_client()