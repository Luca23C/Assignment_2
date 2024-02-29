#!/usr/bin/env python

import rospy
import math
from assignment_2_2023.srv import Navigation, NavigationResponse
from assignment_2_2023.msg import *

"""
.. module:: assignment_2_2023

    :platform: Unix
    :synopsis: Python module for the assignment_2_2023

    :moduleauthor: Luca Cornia

Subscribers to:
    /custom_message
    /reaching_goal/goal

Service:
    /navigation_service

Global variables:
    real_pose_x: float
        Init of current x coordinate position of the robot
    real_pose_y: float
        Init of current y coordinate position of the robot
    real_linear_vel:
        Init of current linear velocity the robot
    real_angular_vel: float
        Init of current angular velocity of the robot
    goal_pose_x: float
        Init of the x coordinate of the goal position
    goal_pose_y: float
        Init of the x coordinate of the goal position
    velocity_array: list of float
        Storage the linear velocity of the robot
"""

real_pose_x = 0
real_pose_y = 0
real_linear_vel = 0
real_angular_vel = 0
goal_pose_x = 0
goal_pose_y = 0
velocity_array = []


def read_info_nav(info):
    """
    Callback of the subscriber which subscribe to the topic /custom_message

    Parameters:
    -----------

    real_pose_x: float
        Storage the current x coordinate of the robot position
    real_pose_y: float
        Storage the current y coordinate of the robot position
    real_linear_vel: float
        Storage the current linear velocity of the robot
    real_angular_vel: float
        Storage the current angular velocity of the robot
    velocity_array: list
        Create linear velocity array
    """
    global real_pose_x, real_pose_y, real_linear_vel, real_angular_vel, velocity_array
    real_pose_x = info.pose_x
    real_pose_y = info.pose_y
    real_linear_vel = info.vel_x
    real_angular_vel = info.vel_z

    velocity_array.append(real_linear_vel)


def read_goal(goal):
    """
    Callback of the subscriber which subscribe the information of the goal chosen

    Parameters:
    -----------

    goal_pose_x: float
        Storage the x coordinate of the goal
    goal_pose_y: float
        Storage the y coordinate of the goal
    """
    global goal_pose_x, goal_pose_y
    goal_pose_x = goal.goal.target_pose.pose.position.x
    goal_pose_y = goal.goal.target_pose.pose.position.y


def nav_info_callback(req):
    """
    Compute the distance between the robot and the goal as well as the average linear velocity of the robot

    Parameters:
    -----------

    req.x: float
        Storage the information of the current x coordinate of the robot position
    req.y: float
        Storage the information of the current y coordinate of the robot position
    req.linear_vel_x: float
        Storage the information of the linear velocity of the robot
    req.angular_vel_z: float
        Storage the information of the angular velocity of the robot
    dist: float
        Euclidean distance from target to actual robot position
    window_size: int
        Define the size of the window, because it has been used the average window technique. This parameter is retrieved from launch file
    moving_avg: list of float
        Storage the average velocity of a specific window
    count: int
        Useful for the average window loop technique
    window: list of float
        List of velocities recorded in a window
    window_avg: float
        Average value of the velocities in that specific window computed before
    avg_speed: float
        Get information about the total average speed of any window

    Returns:
    --------
    dist: float
        Euclidean distance from target to actual robot position
    avg_speed: float
        Average speed of the robot
    """
    global real_pose_x, real_pose_y, real_linear_vel, real_angular_vel, goal_pose_x, goal_pose_y, velocity_array
    req.x = real_pose_x
    req.y = real_pose_y
    req.linear_vel_x = real_linear_vel
    req.angular_vel_z = real_angular_vel

    dist = math.sqrt((goal_pose_x - real_pose_x)**2 + (goal_pose_y - real_pose_y)**2)

    window_size = rospy.get_param("/avg_window")
    moving_avg = []
    count = 0
    while count < len(velocity_array) - window_size + 1:
        window = velocity_array[count : count + window_size]
        window_avg = round(sum(window)/window_size, 3)
        moving_avg.append(window_avg)
        count += 1

    avg_speed = sum(moving_avg)/len(moving_avg)

    return NavigationResponse(dist, avg_speed)


def nav_server():
    """
    Main of my service. The goal is to get the information about the current position and velocity of the robot as well as the position of the goal.
    """
    rospy.init_node('info_nav_server')
    sub_1 = rospy.Subscriber('/custom_message', Info, read_info_nav)
    sub_2 = rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, read_goal)
    serv = rospy.Service('/navigation_service', Navigation, nav_info_callback)
    rospy.spin()


if __name__=='__main__':
    nav_server()