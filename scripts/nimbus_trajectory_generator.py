#!/usr/bin/env python

import roslib
import rospy
import actionlib
import sensor_msgs.msg
import control_msgs.msg
import wpi_jaco_msgs.msg
import trajectory_msgs.msg

#import hlpr_manipulation_utils.manipulator
#import hlpr_manipulation_utils.arm_moveit


# This has been inspired from hlpr_manipulation_utils.Manipulator.Arm
class ArmTrajectoryGenerator:
    def __init__(self, arm_prefix = 'jaco'):
        self.pub_jaco_ang = rospy.Publisher('/jaco_arm/angular_cmd', wpi_jaco_msgs.msg.AngularCommand, queue_size=10, latch=True)
        self.pub_jaco_cart = rospy.Publisher('/jaco_arm/cartesian_cmd', wpi_jaco_msgs.msg.CartesianCommand, queue_size=10, latch=True)

        self._arm_prefix = arm_prefix
        self.arm_joint_names = [self._arm_prefix + "_shoulder_pan_joint", self._arm_prefix + "_shoulder_lift_joint",
                                self._arm_prefix + "_elbow_joint",
                                self._arm_prefix + "_wrist_1_joint", self._arm_prefix + "_wrist_2_joint",
                                self._arm_prefix + "_wrist_3_joint"]

        self.joint_states = [0 for i in range(0, len(self.arm_joint_names))]

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

    # I have fixed the position length. nimbus also publishes the gripper joint states which are extraneous
    def js_cb(self, inState):
        for i in range(0, len(inState.position)):
            self.joint_states[i] = inState.position[i]

        self.last_js_update = rospy.get_time()


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



def vectorDiff(v1,v2):
 error = 0
 l = min(len(v1),len(v2))
 for i in range(0,l):
   diff = (v1[i] - v2[i])
   error += diff*diff
 error = sqrt(error)
 return error

