#! /usr/bin/env python
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
#from chores.msg import DoDishesAction, DoDishesGoal

if __name__ == '__main__':
    rospy.init_node('simple_navigation_goals')
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = 30.0
    goal.target_pose.pose.orientation.w = 13.0

    rospy.loginfo("Sending goal")
    ac.sendGoal(goal)

    ac.waitForResult()
    # Fill in the goal here
    if(ac.get_state() == "SUCCEEDED"):
        rospy.loginfo("Hooray, the base moved 1 meter forward")
    else:
        rospy.loginfo("The base failed to move forward 1 meter for some reason")