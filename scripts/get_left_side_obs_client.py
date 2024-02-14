#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import LeftSideObstacle, LeftSideObstacleResponse


if __name__=='__main__':

    rospy.init_node('get_dist_left_side_client')

    client = rospy.ServiceProxy('/left_side', LeftSideObstacle)
    rospy.wait_for_service('/left_side')

    response = client()

    rospy.loginfo("Distance left side obstacle: ", response.distance)