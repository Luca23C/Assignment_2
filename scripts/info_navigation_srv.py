#!/usr/bin/env python

import rospy
import math
from assignment_2_2023.srv import Navigation, NavigationResponse
from assignment_2_2023.msg import *

# Global variables
real_pose_x = 0
real_pose_y = 0
real_linear_vel = 0
real_angular_vel = 0
goal_pose_x = 0
goal_pose_y = 0
velocity_array = []


def read_info_nav(info):
    global real_pose_x, real_pose_y, real_linear_vel, real_angular_vel, velocity_array
    real_pose_x = info.pose_x
    real_pose_y = info.pose_y
    real_linear_vel = info.vel_x
    real_angular_vel = info.vel_z

    # Create linear velocity array
    velocity_array.append(real_linear_vel)


def read_goal(goal):
    global goal_pose_x, goal_pose_y
    goal_pose_x = goal.goal.target_pose.pose.position.x
    goal_pose_y = goal.goal.target_pose.pose.position.y


def nav_info_callback(req):
    global real_pose_x, real_pose_y, real_linear_vel, real_angular_vel, goal_pose_x, goal_pose_y, velocity_array
    req.x = real_pose_x
    req.y = real_pose_y
    req.linear_vel_x = real_linear_vel
    req.angular_vel_z = real_angular_vel

    # Calculate Euclidean distance from target to actual robot position
    dist = math.sqrt((goal_pose_x - real_pose_x)**2 + (goal_pose_y - real_pose_y)**2)

    # Calculate averege speed of the robot
    window_size = rospy.get_param("/avg_window")    # Get this parameter from launch file
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
    rospy.init_node('info_nav_server')
    sub_1 = rospy.Subscriber('/custom_message', Info, read_info_nav)
    sub_2 = rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, read_goal)
    serv = rospy.Service('/navigation_service', Navigation, nav_info_callback)
    rospy.spin()


if __name__=='__main__':
    nav_server()