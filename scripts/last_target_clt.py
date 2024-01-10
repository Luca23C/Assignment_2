#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import Position, PositionResponse


def position_client():

    rospy.init_node('last_target_position_client')
    client = rospy.ServiceProxy('/last_pose', Position)

    rospy.wait_for_service('/last_pose')
    response = client()

    rospy.loginfo("Last target coordinate are: x: %f, y: %f", response.x_res, response.y_res)


if __name__=='__main__':
    position_client()