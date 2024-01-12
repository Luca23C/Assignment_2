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

class ActionClient:

    # Define global variable for analize the current status of the target
    target_status = None
    target_cancelled = False

    def __init__(self):
        rospy.init_node('bug_ac')
        self.client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.client.wait_for_server()

        # Define subscriber for checking the status of the goal
        self.sub_stat = rospy.Subscriber('/reaching_goal/feedback', String, self.status_callback)

        # Define publisher and subscriber for custom message
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/custom_message', Info, queue_size=10)


    def on_press(self, key):
        if key == keyboard.Key.esc:
            self.target_cancelled = True
            return False  # stop listener
        """ else:
            rospy.logerr("Invalid input. If you want to delete target, please press 'esc'.")
 """

    def status_callback(self, stat):
        self.target_status = stat.feedback.stat

  
    def odom_callback(self, odom_msg):
        # Fill the field of my custom message
        info_msg = Info()
        info_msg.pose_x = odom_msg.pose.pose.position.x
        info_msg.pose_y = odom_msg.pose.pose.position.y
        info_msg.vel_x = odom_msg.twist.twist.linear.x
        info_msg.vel_z = odom_msg.twist.twist.angular.z

        self.pub.publish(info_msg)


    def create_goal(self, x, y):
        # Construction goal message
        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        self.client.send_goal(goal)
        rospy.loginfo("Pose goal sent to the action server.")

        # Allow to delete goal
        self.delete_goal()


    def delete_goal(self):
        print("\nIf you want to delete target, please press 'esc'.")
        # Fuction for listening if key 'esc' is pressed
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        while True:

            timer = self.client.wait_for_result(rospy.Duration(1))
        
            if timer:
                rospy.loginfo(self.target_status)
                listener.stop()
                self.target_cancelled = False
                break

            elif self.target_cancelled:
                self.client.cancel_goal()
                self.target_cancelled = False
                time.sleep(1)                       # Useful for getting the correct status
                rospy.loginfo(self.target_status)
                break


    def start_interface(self):
        while not rospy.is_shutdown():
            try:
                time.sleep(1)
                termios.tcflush(sys.stdin, termios.TCIOFLUSH)   # Updates buffer after deleting target

                # Create new interface
                print('\n\n######################################################################################')
                print("\nWelcome to user interface!\nHere you can choose the target position for the robot.\n")
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

