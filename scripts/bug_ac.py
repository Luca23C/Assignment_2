#!/usr/bin/env python

import rospy
import keyboard
import actionlib
import assignment_2_2023.msg

class ActionClient:
    def __init__(self):
        rospy.init_node('bug_ac')
        self.client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.client.wait_for_server()
            

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
        print("If you want to delete target, please enter 'y'.\nPress 'enter' to refresh terminal when robot reach the goal.")
        request = str(input("Digit your choice: "))
        while True:
            timer = self.client.wait_for_result(rospy.Duration(1))
        
            if timer:
                break

            elif request == 'y':
                self.client.cancel_goal()
                rospy.loginfo("Goal successfully deleted!")
                break


    def start_interface(self):
        while not rospy.is_shutdown():
            try:
                x = float(input("Enter x coordinate: "))
                y = float(input("Enter y coordinate: "))

                self.create_goal(x, y)

            except ValueError:
                rospy.logerr("Invalid input. Please provide valid numbers.")



if __name__ == '__main__':
    try:
        node = ActionClient()
        node.start_interface()
    except rospy.ROSInterruptException:
        pass

