#!/usr/bin/env python3

from robotiq_2f_gripper_control.robotiq_gripper import RobotiqGripper
import time

import rospy
from std_msgs.msg import Int16

gripper = RobotiqGripper()

def callback(data):
    try:
        gripper.move_and_wait_for_pos(data.data, 255, 255)
        rospy.loginfo(f"Gripper moved to position: {data.data}")
    except Exception as e:
        rospy.logerr(f"Error moving gripper: {e}")

if __name__ == '__main__':
    rospy.init_node('robotiq_2d_gripper_control_node', anonymous=True)

    robot_ip = rospy.get_param('~robot_ip', '192.168.1.3')
    port = int(rospy.get_param('~port', '63352'))

    rospy.loginfo("Connecting to gripper...")
    try:
        gripper.connect(robot_ip, port)
        rospy.loginfo("Connected to gripper.")
    except Exception as e:
        rospy.logerr(f"Failed to connect to gripper: {e}")
        rospy.signal_shutdown("Connection error")

    rospy.loginfo("Activating gripper...")
    try:
        gripper.activate(auto_calibrate=False)
        rospy.loginfo("Gripper activated.")
    except Exception as e:
        rospy.logerr(f"Failed to activate gripper: {e}")
        rospy.signal_shutdown("Activation error")

    rospy.Subscriber("/ur_controller/gripperCmd", Int16, callback)

    pub = rospy.Publisher('/gripperCurrentPose', Int16, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    rospy.loginfo("Gripper initialized and running...")
    while not rospy.is_shutdown():
        try:
            pub.publish(gripper.get_current_position())
            rate.sleep()
        except Exception as e:
            rospy.logerr(f"Error getting current position: {e}")

    rospy.spin()

