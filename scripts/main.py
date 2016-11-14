#!/usr/bin/env python

import roslib
import rospy

import rail_manipulation_msgs.msg
import rail_manipulation_msgs.srv
import actionlib
import sensor_msgs.msg
import control_msgs.msg
import wpi_jaco_msgs.msg
import wpi_jaco_msgs.srv
import trajectory_msgs.msg
import robotiq_85_msgs.msg

from math import pi, sqrt

# import hlpr_manipulation_utils.manipulator
# import hlpr_manipulation_utils.arm_moveit


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
        rospy.loginfo('Opening the Gripper...')
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


# This has been inspired from hlpr_manipulation_utils.Manipulator.Arm
class ArmTrajectoryGenerator:
    def __init__(self, arm_prefix = 'jaco', filenameIn = 'Temp.dat'):
        self.pub_jaco_ang = rospy.Publisher('/jaco_arm/angular_cmd', wpi_jaco_msgs.msg.AngularCommand, queue_size=10, latch=True)
        self.pub_jaco_cart = rospy.Publisher('/jaco_arm/cartesian_cmd', wpi_jaco_msgs.msg.CartesianCommand, queue_size=10, latch=True)


        #Added by me
        ######

        self.filename = filenameIn
        self.fileout = open(self.filename, 'wb')

        self.EEPose = wpi_jaco_msgs.srv.JacoFKResponse
        self.gripperPosition = None
        rospy.wait_for_service('/jaco_arm/kinematics/fk')
        self.compute_fk = rospy.ServiceProxy('/jaco_arm/kinematics/fk', wpi_jaco_msgs.srv.JacoFK)
        rospy.Subscriber('/gripper/stat', robotiq_85_msgs.msg.GripperStat, self.gripper_cb)

        ####

        self._arm_prefix = arm_prefix
        self.arm_joint_names = [self._arm_prefix + "_shoulder_pan_joint", self._arm_prefix + "_shoulder_lift_joint",
                                self._arm_prefix + "_elbow_joint",
                                self._arm_prefix + "_wrist_1_joint", self._arm_prefix + "_wrist_2_joint",
                                self._arm_prefix + "_wrist_3_joint"]

        self.joint_states = [0 for i in range(0, len(self.arm_joint_names))]
        self.joint_effort = [0 for i in range(0, len(self.arm_joint_names))]

        rospy.Subscriber('/jaco_arm/joint_states', sensor_msgs.msg.JointState, self.js_cb)
        self.last_js_update = None

        self.smooth_joint_trajectory_client = actionlib.SimpleActionClient(
            '/jaco_arm/joint_velocity_controller/trajectory', control_msgs.msg.FollowJointTrajectoryAction)

        if (self.smooth_joint_trajectory_client.wait_for_server()):
            self.traj_connection = True
        else:
            self.traj_connection = False

        print self.traj_connection

        self.angular_cmd = wpi_jaco_msgs.msg.AngularCommand()
        self.angular_cmd.armCommand = True
        self.angular_cmd.fingerCommand = False
        self.angular_cmd.repeat = True

        self.cartesian_cmd = wpi_jaco_msgs.msg.CartesianCommand()
        self.cartesian_cmd.armCommand = True
        self.cartesian_cmd.fingerCommand = False
        self.cartesian_cmd.repeat = True

        # self._init_tuck_poses() # I havent copied the definition yet

        self.gc_connection = False



        # self.arm_planner = hlpr_manipulation_utils.arm_moveit.ArmMoveIt()  # This call has some problems


    def js_cb(self, inState):
        for i in range(0, len(inState.position)):
            self.joint_states[i] = inState.position[i]
            self.joint_effort[i] = inState.effort[i]

        try:
            self.EEPose = self.compute_fk(self.joint_states)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        print self.EEPose
        self.last_js_update = rospy.get_time()

        output_string = str(self.last_js_update) + ' ' + str(self.joint_states)[1:-1] + ' ' + \
                        str(self.EEPose.handPose.pose.position.x) + ' ' + str(self.EEPose.handPose.pose.position.y) + ' ' + str(self.EEPose.handPose.pose.position.z) + ' ' + \
                        str(self.EEPose.handPose.pose.orientation.x) + ' ' + str(self.EEPose.handPose.pose.orientation.y) + ' ' + str(self.EEPose.handPose.pose.orientation.z) + str(self.EEPose.handPose.pose.orientation.w)+ ' ' + \
                        str(self.gripperPosition) + \
                        str(self.joint_effort)[1:-1] + '\n'

        self.fileout.write(output_string)



    def gripper_cb(self, gripperStat):
        self.gripperPosition = gripperStat.position

        print self.gripperPosition


    def get_pos(self):
        return self.joint_states

    def ang_pos_cmd(self, angles):
        if not len(angles) == len(self.arm_joint_names):
            print "Number of desired joint angles does not match the number of available joints"
            return
        self.angular_cmd.position = True
        self.angular_cmd.joints = angles
        self.pub_jaco_ang.publish(self.angular_cmd)

    def ang_vel_cmd(self, velocities):
        if not len(velocities) == len(self.arm_joint_names):
            print "Number of desired joint velocities does not match the number of available joints"
            return
        self.angular_cmd.position = False
        self.angular_cmd.joints = velocities
        self.pub_jaco_ang.publish(self.angular_cmd)


    def cart_pos_cmd(self, pose):
        if not len(pose) == 6:
            print "Not enough pose parameters specified"
            return
        self.cartesian_cmd.position = True
        self.cartesian_cmd.arm.linear.x = pose[0]
        self.cartesian_cmd.arm.linear.y = pose[1]
        self.cartesian_cmd.arm.linear.z = pose[2]

        self.cartesian_cmd.arm.angular.x = pose[3]
        self.cartesian_cmd.arm.angular.y = pose[4]
        self.cartesian_cmd.arm.angular.z = pose[5]

        self.pub_jaco_cart.publish(self.cartesian_cmd)

    def cart_pos_cmd(self, translation, rotation):
        if not len(translation) == 3:
            print "Not enough translations specified"
            return
        if not len(rotation) == 3:
            print "Not enough rotations specified"
            return
        pose = translation + rotation
        self.cart_pos_cmd(pose)


    def cart_vel_cmd(self, vels):
        if not len(vels) == 6:
            print "Not enough velocities specified"
            return
        self.cartesian_cmd.position = False
        self.cartesian_cmd.arm.linear.x = vels[0]
        self.cartesian_cmd.arm.linear.y = vels[1]
        self.cartesian_cmd.arm.linear.z = vels[2]

        self.cartesian_cmd.arm.angular.x = vels[3]
        self.cartesian_cmd.arm.angular.y = vels[4]
        self.cartesian_cmd.arm.angular.z = vels[5]

        self.pub_jaco_cart.publish(self.cartesian_cmd)

    def cart_vel_cmd(self, translation, rotation):
        if not len(translation) == 3:
            print "Not enough translation velocities specified"
            return
        if not len(rotation) == 3:
            print "Not enough rotation velocities specified"
            return
        vels = translation + rotation
        self.cart_pos_cmd(vels)

    def ang_cmd_loop(self, angles, rate=10, iterations=5):
        rrate = rospy.Rate(rate)
        for i in range(0, iterations):
            self.ang_pos_cmd(angles)
            rrate.sleep()

    def ang_cmd_wait(self, angles, epsilon=0.05, maxIter=50, rate=10):
        error = epsilon + 1
        epsilon = 5
        iterNum = 0
        # self.ang_cmd_loop(angles,rate)
        self.ang_pos_cmd(angles)
        rrate = rospy.Rate(rate)
        while error > epsilon and iterNum < maxIter:
            error = vectorDiff(self.joint_states, angles)
            iterNum += 1
            rrate.sleep()

        if iterNum == maxIter:
            return False
        return True

    def sendWaypointTrajectory(self, waypoints, durations=0., vels=0., accs=0., effs=0.):
        if not self.ang_cmd_wait(waypoints[0]):
            print 'Cannot go to the first point in the trajectory'
            return None
            #    else:
            #      print 'Went to first'

        if not self.traj_connection:
            print 'Action server connection was not established'
            return None
        joint_traj = trajectory_msgs.msg.JointTrajectory()
        joint_traj.joint_names = self.arm_joint_names

        if not durations == 0:
            if not len(durations) == waypoints:
                raise Exception('The number of duration points is not equal to the number of provided waypoints')
        if not vels == 0:
            if not len(vels) == waypoints:
                raise Exception('The number velocity points is not equal to the number of provided waypoints')
        if not accs == 0:
            if not len(accs) == waypoints:
                raise Exception('The number acceleration points is not equal to the number of provided waypoints')
        if not effs == 0:
            if not len(effs) == waypoints:
                raise Exception('The number effort points is not equal to the number of provided waypoints')

        if not effs == 0:
            if not (vels == 0 and accs == 0):
                raise Exception('Cannot specify efforts with velocities and accelerations at the same time')
        if (not accs == 0) and vels == 0:
            raise Exception('Cannot specify accelerations without velocities')

        total_time_from_start = 0.5
        for t in range(0, len(waypoints)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()

            waypoint = waypoints[t]
            if not len(waypoint) == len(joint_traj.joint_names):
                raise Exception(
                    'The number of provided joint positions is not equal to the number of available joints for index: '
                    + str(t))
            point.positions = waypoint

            if not vels == 0.:
                velocity = vels[t]
                if not len(velocity) == len(joint_traj.joint_names):
                    raise Exception(
                        'The number of provided joint velocities is not equal to the number of available joints for index: ' + str(
                            t))
                point.velocities = velocity

            if not accs == 0.:
                acceleration = accs[t]
                if not len(acceleration) == len(joint_traj.joint_names):
                    raise Exception(
                        'The number of provided joint accelerations is not equal to the number of available joints for index: ' + str(
                            t))
                point.accelerations = acceleration

            if not effs == 0.:
                effort = effs[t]
                if not len(effort) == len(joint_traj.joint_names):
                    raise Exception(
                        'The number of provided joint efforts is not equal to the number of available joints for index: ' + str(
                            t))
                point.effort = effort

            if not durations == 0.:
                point.duration = durations

            # Deal with increasing time for each trajectory point
            point.time_from_start = rospy.Duration(total_time_from_start)
            total_time_from_start = total_time_from_start + 1.0

            # Set the points
            joint_traj.points.append(point)

        traj_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj_goal.trajectory = joint_traj

        self.smooth_joint_trajectory_client.send_goal(traj_goal)
        self.smooth_joint_trajectory_client.wait_for_result()
        return self.smooth_joint_trajectory_client.get_result()


def vectorDiff(v1, v2):
    error = 0
    l = min(len(v1), len(v2))
    for i in range(0, l):
        diff = (v1[i] - v2[i])
        error += diff * diff
    error = sqrt(error)
    return error

#
# class Recorder:
#     def __init__(self, filenameIn='Temp.dat'):
#         self.filename = filenameIn
#
#         self.last_js_update = None
#         self.joint_states = None
#
#
#
#     def js_cb(self, inState):
#         for i in range(0, len(inState.position)):
#             self.joint_states[i] = inState.position[i]
#
#
#
#         self.last_js_update = rospy.get_time()





if __name__ == '__main__':
    rospy.init_node('primitive_action_client')
    translationDistance = 0.15
    rotationAngle = 1.5708
    nameIn = 'Right2'
    primitiveActionClient = PrimitiveActionClient(translationDistance, rotationAngle)
    armTraGen = ArmTrajectoryGenerator(arm_prefix='jaco', filenameIn=(nameIn + '.dat'))



    homePos = [-3.1363267851663794, 4.049387303842302, 0.8741868222972268,
               0.9659132141954717, 1.858019715098542, 0.8635215945007593]

    handlePosMid = [-2.6277471728516915, 4.15765385690173, 1.0027724955256316,
                    1.4330565699515623, 1.8879208888696246, 0.9407003357863918]

    handlePosLeft1 = [-2.7467960662165223, 4.378151610935385, 1.9107107567399233,
                      0.45758117599362025, 2.1945211709485775, 1.4463970096420418]

    handlePosLeft2 = [-2.739850009461378, 4.197578370067544, 1.3345816612720294,
                      0.994237663443173, 1.7966877809842896, 1.255771933555805]

    handlePosRight2 = [-2.354086864505656, 4.101280062946932, 0.7913477984307398,
                       2.0100365453877367, 2.0283256710158013, 0.8485722726167357]

    handlePosRight1 = [-1.997455506082093, 4.127843496923819, 0.7323045849573016,
                       2.5844012966334526, 2.1795570019414767, 0.8036523350362057]

    name_dict = {'home': homePos,
                 'Left1':handlePosLeft1, 'Left2':handlePosLeft2,
                 'Right1':handlePosRight1, 'Right2':handlePosRight2,
                 'Mid': handlePosMid}

    primitiveActionClient.executeOpenGripper()
    rospy.sleep(1.0)


    armTraGen.ang_pos_cmd(name_dict[nameIn])
    rospy.sleep(3.0)

    primitiveActionClient.executeCloseGripper()
    rospy.sleep(1.0)
    primitiveActionClient.executeBackwardMotion()

    rospy.sleep(3.0)
    primitiveActionClient.executeOpenGripper()
    rospy.sleep(3.0)

    #armTraGen.fileout.close()




    #
    #
    # app1_wp0 = [-1.65, 3.68, 1.12, -2.13, 1.48, 2.10]
    # app1_wp1 = [-1.49, 4.00, 1.47, -1.74, 1.25, 1.96]
    # app1_wp2 = [-1.23, 4.50, 0.95, -2.31, 1.82, 1.96]
    # app1_wp3 = [-1.21, 4.76, 0.83, -2.60, 2.56, 1.63]

    # app1 = [app1_wp0, app1_wp0, app1_wp0, app1_wp0]
    #
    #
    # amtra.sendWaypointTrajectory(app1)

    rospy.spin()


