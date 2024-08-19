#!/usr/bin/env python3

from robotiq_2f_gripper_control.robotiq_gripper import RobotiqGripper
import rospy
import actionlib
import robotiq_2f_gripper_control.msg

class GripperAction(object):
    __feedback = robotiq_2f_gripper_control.msg.gripActionFeedback()
    __result = robotiq_2f_gripper_control.msg.gripActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, robotiq_2f_gripper_control.msg.gripAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.gripper = RobotiqGripper()
        try:
            self.gripper.connect("localhost", 65500)
            self.gripper.activate(auto_calibrate=False)
        except Exception as e:
            rospy.logerr('Failed to initialize gripper: %s' % str(e))
            self.gripper = None

        rospy.loginfo("Robotiq Action Server running")

    def execute_cb(self, goal):
        if self.gripper is None:
            rospy.logerr('Gripper is not initialized properly.')
            self.__result.result = False
            self._as.set_aborted(self.__result)
            return

        r = rospy.Rate(1)
        success = True

        self.__feedback.current = self.gripper.get_current_position()
        rospy.loginfo('%s: Executing, moving gripper to %i' % (self._action_name, goal.position))

        self.gripper.move(goal.position, goal.speed, goal.force)

        while(self.gripper.get_current_position() != goal.position):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                # TODO stop the gripper
                break
                
            self.__feedback.current_position = self.gripper.get_current_position()
            self._as.publish_feedback(self.__feedback)
            r.sleep()

        if success:
            self.__result.result = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self.__result)
        else:
            self.__result.result = False
            self._as.set_aborted(self.__result)

if __name__ == '__main__':
    rospy.init_node('gripper_action_server')
    server = GripperAction(rospy.get_name())
    rospy.spin()
