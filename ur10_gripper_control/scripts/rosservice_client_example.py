#!/usr/bin/env python

import rospy
from ur10_gripper_control.srv import MoveRobot
from geometry_msgs.msg import Pose

def move_robot_client():
    rospy.init_node('ur10_gripper_controller_client')

    # Wait for the service to become available
    rospy.wait_for_service('ur10_gripper_controller/MoveRobot')
    
    try:
        # Create a service proxy
        move_robot = rospy.ServiceProxy('ur10_gripper_controller/MoveRobot', MoveRobot)
        
        # Create an instance of the Pose message
        pose = Pose()
        
        # Create a request to the service
        response = move_robot(pose)
        
        # Check the result and print appropriate message
        if response.result:
            rospy.loginfo("Result: %s", response.result)
        else:
            rospy.logwarn("Service call failed")
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    move_robot_client()