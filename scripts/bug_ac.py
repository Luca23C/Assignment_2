#!/usr/bin/env python

import rospy
import actionlib
import assignment_2_2023.msg

class ActionClient:
    def __init__(self):
        rospy.init_node('bug_ac')
        self.client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.client.wait_for_server()

    def send_goal(self, x, y):
        # Construction goal message
        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "Rover"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        rospy.loginfo("Pose goal sent to the action server.")

        # Possibility to cancel goal
        timer = self.client.wait_for_result(rospy.Duration(30))

        if timer == True:
            state = self.client.get_result()
            rospy.loginfo("Position reached"+str(state))

        else:
            rospy.loginfo("Would you like to cancel the goal?")
            request = str(input("Please write 'y' or 'n' "))

            if request == 'y':
                self.client.cancel_goal()

            elif request == 'n':
                self.client.wait_for_result()

    def start_interface(self):
        while not rospy.is_shutdown():
            try:
                x = float(input("Enter X coordinate: "))
                y = float(input("Enter Y coordinate: "))

                self.send_goal(x, y)
            except ValueError:
                rospy.logerr("Invalid input. Please provide valid numbers.")



if __name__ == '__main__':
    try:
        node = ActionClient()
        node.start_interface()
    except rospy.ROSInterruptException:
        pass

