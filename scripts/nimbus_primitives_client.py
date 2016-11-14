#!/usr/bin/env python

import roslib
import rospy
import rail_manipulation_msgs.msg
import rail_manipulation_msgs.srv




class PrimitiveActionClient:
    def __init__(self, distIn=0.1, angleIn=1.5708):
        self.dist = distIn
        self.angle = angleIn
        self.primitiveClient = actionlib.SimpleActionClient('nimbus_moveit/primitive_action', rail_manipulation_msgs.msg.PrimitiveAction)
        self.primitiveClient.wait_for_server()
        self.armClient = actionlib.SimpleActionClient('/nimbus_moveit/common_actions/arm_action', rail_manipulation_msgs.msg.ArmAction)
        self.armClient.wait_for_server()
        self.gripperClient = actionlib.SimpleActionClient('/gripper_actions/gripper_manipulation', rail_manipulation_msgs.msg.GripperAction)
        self.gripperClient.wait_for_server()

    def executeResetArm(self):
        rospy.loginfo('Moving Arm to Reset Position...')
        motionGoal = rail_manipulation_msgs.msg.ArmGoal(action=1)
        self.armClient.send_goal(motionGoal)
        self.armClient.wait_for_result()
        result = self.armClient.get_result()
        rospy.loginfo('Result: %d' % result.success)

    def executeCloseGripper(self):
        rospy.loginfo('Closing the Gripper...')
        motionGoal = rail_manipulation_msgs.msg.GripperGoal(close=True)
        self.gripperClient.send_goal(motionGoal)
        self.gripperClient.wait_for_result()
        result = self.gripperClient.get_result()
        rospy.loginfo('Result: %d' % result.success)

    def executeOpenGripper(self):
        rospy.loginfo('Closing the Gripper...')
        motionGoal = rail_manipulation_msgs.msg.GripperGoal(close=False)
        self.gripperClient.send_goal(motionGoal)
        self.gripperClient.wait_for_result()
        result = self.gripperClient.get_result()
        rospy.loginfo('Result: %d' % result.success)


    def executeForwardMotion(self):
        rospy.loginfo('Executing Forward Motion...')
        motionGoal = rail_manipulation_msgs.msg.PrimitiveGoal(primitive_type=0, axis=1, distance=self.dist)
        self.primitiveClient.send_goal(motionGoal)
        self.primitiveClient.wait_for_result()
        result = self.primitiveClient.get_result()
        rospy.loginfo('Result: %d' % result.completion)

    def executeBackwardMotion(self):
        rospy.loginfo('Executing Backward Motion...')
        motionGoal = rail_manipulation_msgs.msg.PrimitiveGoal(primitive_type=0, axis=1, distance=(-1)*self.dist)
        self.primitiveClient.send_goal(motionGoal)
        self.primitiveClient.wait_for_result()
        result = self.primitiveClient.get_result()
        rospy.loginfo('Result: %d' % result.completion)

    def executeLeftMotion(self):
        rospy.loginfo('Executing Left Motion...')
        motionGoal = rail_manipulation_msgs.msg.PrimitiveGoal(primitive_type=0, axis=0, distance=(-1)*self.dist)
        self.primitiveClient.send_goal(motionGoal)
        self.primitiveClient.wait_for_result()
        result = self.primitiveClient.get_result()
        rospy.loginfo('Result: %d' % result.completion)

    def executeRightMotion(self):
        rospy.loginfo('Executing Right Motion...')
        motionGoal = rail_manipulation_msgs.msg.PrimitiveGoal(primitive_type=0, axis=0, distance=self.dist)
        self.primitiveClient.send_goal(motionGoal)
        self.primitiveClient.wait_for_result()
        result = self.primitiveClient.get_result()
        rospy.loginfo('Result: %d' % result.completion)

    def executeUpwardMotion(self):
        rospy.loginfo('Executing Upward Motion...')
        motionGoal = rail_manipulation_msgs.msg.PrimitiveGoal(primitive_type=0, axis=2, distance=self.dist)
        self.primitiveClient.send_goal(motionGoal)
        self.primitiveClient.wait_for_result()
        result = self.primitiveClient.get_result()
        rospy.loginfo('Result: %d' % result.completion)

    def executeDownwardMotion(self):
        rospy.loginfo('Executing Downward Motion...')
        motionGoal = rail_manipulation_msgs.msg.PrimitiveGoal(primitive_type=0, axis=2, distance=(-1)*self.dist)
        self.primitiveClient.send_goal(motionGoal)
        self.primitiveClient.wait_for_result()
        result = self.primitiveClient.get_result()
        rospy.loginfo('Result: %d' % result.completion)

    def executeRotateCWMotion(self):
        rospy.loginfo('Executing Clockwise Motion...')
        motionGoal = rail_manipulation_msgs.msg.PrimitiveGoal(primitive_type=1, axis=0, distance=(-1)*self.angle)
        self.primitiveClient.send_goal(motionGoal)
        self.primitiveClient.wait_for_result()
        result = self.primitiveClient.get_result()
        rospy.loginfo('Result: %d' % result.completion)

    def executeRotateCCWMotion(self):
        rospy.loginfo('Executing Counter-Clockwise Motion...')
        motionGoal = rail_manipulation_msgs.msg.PrimitiveGoal(primitive_type=1, axis=0, distance=self.angle)
        self.primitiveClient.send_goal(motionGoal)
        self.primitiveClient.wait_for_result()
        result = self.primitiveClient.get_result()
        rospy.loginfo('Result: %d' % result.completion)
