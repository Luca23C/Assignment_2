#!/usr/bin/env python

import rospy
from assignment_2_2023.msg import *
from assignment_2_2023.srv import Position, PositionResponse

"""
.. module:: assignment_2_2023

    :platform: Unix
    :synopsis: Python module for the assignment_2_2023

    :moduleauthor: Luca Cornia

Subscriber:
    /reaching_goal/goal

Service:
    /last_pose
"""


x = 0
y = 0


def read_last_goal(goal):
    """
    Allow to get information about the last goal selected

    Parameters
    ----------

    x: float
        Storage the information of the x coordinate subscribed by the topic /reaching_goal/goal
    y: float
        Storage the information of the y coordinate subscribed by the topic /reaching_goal/goal
    """
    global x, y
    x = goal.goal.target_pose.pose.position.x
    y = goal.goal.target_pose.pose.position.y


def get_pose(req):
    """
    Callback of my service which retrive the information requested about the last goal previously sent

    Parameters
    ----------

    req.x: float
        Contain the information of the previously x goal coordinate
    req.y: float
        Contain the information of the previously y goal coordinate
    """
    global x, y
    req.x = x
    req.y = y
    return PositionResponse(req.x, req.y)

def position_server():
    """
    Main function of my service. The goal is to retrieve the information about the last goal chosen by user 
    """
    rospy.init_node('last_target_position_server')
    sub = rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, read_last_goal)
    serv = rospy.Service('/last_pose',Position, get_pose)
    rospy.spin()


if __name__=='__main__':
    position_server()