#!/usr/bin/env python

import rospy
import time
import sys
import termios
from pynput import keyboard
import actionlib
import assignment_2_2023.msg
from std_msgs.msg import String
from assignment_2_2023.msg import Info
from nav_msgs.msg import Odometry
from assignment_2_2023.srv import Position, PositionResponse
from sensor_msgs.msg import LaserScan
from assignment_2_2023.msg import ClosestObstacle
from assignment_2_2023.msg import FeetPose
from assignment_2_2023.srv import DeleteGoal, DeleteGoalResponse


class ActionClient:

    # Define global variable for analize the current status of the target
    target_status = None
    target_cancelled = False

    start = True    # Useful for ex5 FAC SIMILE


    def __init__(self):
        rospy.init_node('bug_ac')
        self.client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.client.wait_for_server()

        # Define subscriber for checking the status of the goal
        self.sub_stat = rospy.Subscriber('/reaching_goal/feedback', String, self.status_callback)

        # Define publisher and subscriber for custom message
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/custom_message', Info, queue_size=10)

        # Define service client that show the previous target goal (FAC SIMILE ex5)
        self.last_goal_client = rospy.ServiceProxy('/last_pose', Position)

        # Reading closest obstacle within laserscan sensor (FAC SIMILE ex5)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.laser_pub = rospy.Publisher('/closest_obstacle', ClosestObstacle, queue_size=10)

        # Publisher for publish the robot's position in feet (EXAM test 1)
        self.pub_feet = rospy.Publisher('/feet_pose', FeetPose, queue_size=10)

        # Service server for deleting goal (EXAM test 1)
        self.delete_goal_service = rospy.Service('/delete_goal', DeleteGoal, self.delete_goal_srv_clbk)

        # Service client for deleting current goal (EXAM test 1)
        self.delete_goal_client = rospy.ServiceProxy('/delete_goal', DeleteGoal)



    def on_press(self, key):
        # Check if is pressed 'esc' key on the keyboard 
        if key == keyboard.Key.esc:
            self.target_cancelled = True
            return False  # stop listener

        """ else:
            rospy.logerr("Invalid input. If you want to delete target, please press 'esc'.")
 """


    def status_callback(self, stat):
        self.target_status = stat.feedback.stat


    # FAC SIMILE ex5
    def laser_callback(self, laser):
        obs = ClosestObstacle()
        distances = laser.ranges
        close = min(distances)
        obs.distance = close

        self.laser_pub.publish(obs)

    
    # EXAM test 1
    def delete_goal_srv_clbk(self, req):
        if req.pressed:
            self.client.cancel_goal()
            correctness = "Goal successfully deleted!"
        
        return DeleteGoalResponse(correctness)

        
  
    def odom_callback(self, odom_msg):
        # Fill the field of my custom message
        info_msg = Info()
        info_msg.pose_x = odom_msg.pose.pose.position.x
        info_msg.pose_y = odom_msg.pose.pose.position.y
        info_msg.vel_x = odom_msg.twist.twist.linear.x
        info_msg.vel_z = odom_msg.twist.twist.angular.z

        # Publish robot's position in feet
        feet = FeetPose()
        x_meter = odom_msg.pose.pose.position.x
        y_meter = odom_msg.pose.pose.position.y

        feet.x_feet = x_meter*3.28
        feet.y_feet = y_meter*3.28

        self.pub.publish(info_msg)
        self.pub_feet.publish(feet)


    def create_goal(self, x, y):
        # Construction goal message
        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        self.client.send_goal(goal)
        rospy.loginfo("Pose goal sent to the action server.")
        self.start = False      # Useful for ex5 FAC SIMILE

        # Allow to delete goal
        self.delete_goal()


    def delete_goal(self):
        print("\nIf you want to delete target, please press 'esc'.")
        # Fuction for listening if key 'esc' is pressed
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        while True:

            # Wait for the robot to reach the target
            timer = self.client.wait_for_result(rospy.Duration(1))
        
            if timer:
                rospy.loginfo(self.target_status)
                listener.stop()                     # stop listener function
                self.target_cancelled = False
                break

            elif self.target_cancelled:
                #self.client.cancel_goal()

                # Exam test 1
                response = self.delete_goal_client(True)
                rospy.loginfo(response.deleted)

                self.target_cancelled = False
                time.sleep(1)                       # Useful for getting the correct status
                rospy.loginfo(self.target_status)
                break


    def start_interface(self):
        while not rospy.is_shutdown():
            try:
                time.sleep(2)                       # To avoid to miss the interface insde the other system information line when the simulation starts
                termios.tcflush(sys.stdin, termios.TCIOFLUSH)   # Updates buffer after deleting target

                # Create new interface
                print('\n\n######################################################################################')
                print("\nWelcome to user interface!\nHere you can choose the target position for the robot.\n")

                # FAC SIMILE ex5
                response = self.last_goal_client()
                if self.start != True:
                    rospy.loginfo("Last target coordinate are: x: %f, y: %f", response.x_res, response.y_res)

                x = float(input("Enter x coordinate: "))
                y = float(input("Enter y coordinate: "))

                # Send coordinates for creating goal
                self.create_goal(x, y)

            except ValueError:
                rospy.logerr("Invalid input. Please provide valid numbers.")


if __name__ == '__main__':
    try:
        node = ActionClient()
        node.start_interface()

    except rospy.ROSInterruptException:
        pass

