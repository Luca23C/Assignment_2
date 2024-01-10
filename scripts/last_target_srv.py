#!/usr/bin/env python

import rospy
from assignment_2_2023.msg import *
from assignment_2_2023.srv import Position, PositionResponse

x = 0
y = 0

def read_last_goal(goal):
    global x, y
    x = goal.goal.target_pose.pose.position.x
    y = goal.goal.target_pose.pose.position.y

def get_pose(req):
    global x, y
    req.x = x
    req.y = y
    return PositionResponse(req.x, req.y)

def position_server():
    rospy.init_node('last_target_position_server')
    sub = rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, read_last_goal)
    serv = rospy.Service('/last_pose',Position, get_pose)
    rospy.spin()


if __name__=='__main__':
    position_server()