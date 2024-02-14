#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import LeftSideObstacle, LeftSideObstacleResponse
from sensor_msgs.msg import LaserScan


def laser_callback(laser):
    global distances
    distances = laser.ranges

def get_left_side(req):
    global distances
    list_of_values = distances

    right = list_of_values[0:143]
    fright = list_of_values[144:278]
    front = list_of_values[288:43]
    fleft = list_of_values[432:575]
    left = list_of_values[576:719]
    dis = min(left)
    print(dis)

    return LeftSideObstacleResponse(dis)
 

if __name__=='__main__':
    rospy.init_node('get_dist_left_side_server')
    laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)
    serv = rospy.Service('/left_side', LeftSideObstacle, get_left_side)
    rospy.spin()