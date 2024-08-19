#!/usr/bin/env python3

import rospy
import actionlib
import robotiq_2f_gripper_control.msg

def feedback_cb(feedback):
    rospy.loginfo("Feedback: Current Position - %i" % feedback.current_position)

def done_cb(status, result):
    if status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Action Succeeded: Result - %s" % result.result)
    else:
        rospy.loginfo("Action Failed with status: %d" % status)

def main():
    rospy.init_node('gripper_action_client')

    # Create an action client
    client = actionlib.SimpleActionClient('gripper_action_server', robotiq_2f_gripper_control.msg.gripAction)

    # Wait for the action server to start
    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()
    rospy.loginfo("Action server started, sending goal...")

    # Create a goal to send
    goal = robotiq_2f_gripper_control.msg.gripGoal()
    goal.position = 100  # Desired position
    goal.speed = 50      # Speed
    goal.force = 20      # Force

    # Send the goal and register callbacks
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    rospy.loginfo("Result: %s" % client.get_result().result)

if __name__ == '__main__':
    main()