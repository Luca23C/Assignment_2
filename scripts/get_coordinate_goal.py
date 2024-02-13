#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import Coordinate_goal, Coordinate_goalResponse
from assignment_2_2023.msg import PlanningActionGoal


def callback(msg):
    global x, y, z
    x = msg.goal.target_pose.pose.position.x
    y = msg.goal.target_pose.pose.position.y
    z = msg.goal.target_pose.pose.position.z

def callback_coord(req):
    global x, y, z
    return Coordinate_goalResponse(x, y, z)


if __name__=='__main__':
    rospy.init_node("get_coordinate_goal_service")
    sub = rospy.Subscriber("/reaching_goal/goal", PlanningActionGoal, callback)
    srv = rospy.Service("/get_coord", Coordinate_goal, callback_coord)
    x = 0
    y = 0
    z = 0

    rospy.spin()