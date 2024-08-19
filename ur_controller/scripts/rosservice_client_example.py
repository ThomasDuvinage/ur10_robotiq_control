#!/usr/bin/env python

import rospy
from ur_controller.srv import MoveRobot
from geometry_msgs.msg import Pose

def move_robot_client():
    rospy.init_node('ur_controllerler_client')

    # Wait for the service to become available
    rospy.wait_for_service('ur_controllerler/MoveRobot')
    
    try:
        # Create a service proxy
        move_robot = rospy.ServiceProxy('ur_controllerler/MoveRobot', MoveRobot)
        
        # Create an instance of the Pose message
        pose = Pose()
        # Edit pose here : 
        # pose.position.x = ...
        # pose.orientation.x = ... 
        
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