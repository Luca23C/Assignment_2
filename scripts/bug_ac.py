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

"""
.. module:: assignment_2_2023
    :platform Unix
    :synopsis: Python module for the assignment_2_2023

.. moduleauthor:: Luca Cornia

ROS Action Client for Controlling a Rover

Subscribes to:
    /reaching_goal/feedback
    /odom

Publishes to:
    /custom_message

Client:
    reaching_goal

"""

class ActionClient:
    """
    A class used to achieve some goal for the robot

    ...

    Attributes
    ----------
    target_status: str
        A string to print on terminal the status of the robot when the target is reached or cancelled
    target_cancelled: bool
        A variable used when the user press 'esc' button for deleting goal


    Methods
    ------
    on_press(key)
        Listen if the user has been pressed the 'esc' button on the keyboard
    status_callback(stat)
        Subscribe from a topic /reaching_goal/feedback for analizing the current state of the robot
    odom_callback(odom_msg)
        Subscribe from a topic /odom for publishing a custom message on a topic called /custom_message
    create_goal(x, y)
        Allow to create the goal message and send this to the action server
    delete_goal()
        Allow to delete the goal
    start_interface()
        Display on terminal the user interface which allow to set a target
    """

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
        """
        Check if is pressed 'esc' key on the keyboard

        Parameters:
        -----------
        key: str
            It rapresents all type of key pressed on the keyboard
        target_cancelled: bool
            Value setted true when the key pressed is equal to esc key
        
        Return
        ------
        The return false allow to interrupt the keyboard listener

        """
        if key == keyboard.Key.esc:
            self.target_cancelled = True
            return False

        #else:
            #rospy.logerr("Invalid input. If you want to delete target, please press 'esc'.")



    def status_callback(self, stat):
        """
        Useful for getting information about the status of the robot

        Parameter
        ---------
        target_status: str
            Storage the information about the current state of the robot
        """
        self.target_status = stat.feedback.stat

  
    def odom_callback(self, odom_msg):
        """
        Get information about the /odom topic for filling the field of my custom message

        Parameters
        ----------

        info_msg.pose_x: float
            Storage the information about the x cartesian position of the robot
        info_msg.pose_y: float
            Storage the information about the y cartesian position of the robot
        info_msg.vel_x: float
            Storage the information about the linear velocity along x-axis of the robot
        info_msg.vel_z: float
            Storage the information about the angular velocity around z-axis of the robot
        """

        info_msg = Info()
        info_msg.pose_x = odom_msg.pose.pose.position.x
        info_msg.pose_y = odom_msg.pose.pose.position.y
        info_msg.vel_x = odom_msg.twist.twist.linear.x
        info_msg.vel_z = odom_msg.twist.twist.angular.z

        self.pub.publish(info_msg)


    def create_goal(self, x, y):
        """
        Create and send to the action server the goal messsage and then that function call the delete_goal function

        Parameters
        ----------

        goal.target_pose.pose.position.x: float
            Assign the x coordinate chosen by the user 
        goal.target_pose.pose.position.y: float
            Assign the y coordinate chosen by the user 
        """
        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        self.client.send_goal(goal)
        rospy.loginfo("Pose goal sent to the action server.")

        # Allow to delete goal
        self.delete_goal()


    def delete_goal(self):
        """
        Activate a listener function which is involved to literally listen if a particular key is pressed

        Parameters
        ----------

        timer: bool
            Get information about the result of a target previously sent
        target_cancelled: bool
            Reset this variable to the initial configuration
        """

        print("\nIf you want to delete target, please press 'esc'.")

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
                self.client.cancel_goal()
                self.target_cancelled = False
                time.sleep(1)                       # Useful for getting the correct status
                rospy.loginfo(self.target_status)
                break


    def start_interface(self):
        """
        Start the user interface where it is possible to set the x and y coordinate of the target desired

        Parameters
        ----------

        x: float
            Storage the x coordinate chosen by the user
        y: float
            Storage the y coordinate chose by the user
        """
        while not rospy.is_shutdown():
            try:
                time.sleep(2)                       # To avoid to miss the interface insde the other system information line when the simulation starts
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

