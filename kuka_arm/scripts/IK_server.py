#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        # Step 1: DH parameters
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Arm length constants
        a1 = 0.35
        d1 = 0.33 + 0.42
        a2 = 1.25
        a3 = -0.054
        d4 = 0.96 + 0.54
        d7 = 0.193 + 0.11

        dh_table = {alpha0: 0, a0: 0, d1: d1, q1: q1,
                    alpha1: -pi/2, a1: a1, d2: 0, q2: q2 - pi/2,
                    alpha2: 0, a2: a2, d3: 0, q3: q3,
                    alpha3: -pi/2, a3: a3, d4: d4, q4: q4,
                    alpha4: pi/2, a4: 0, d5: 0, q5: q5,
                    alpha5: -pi/2, a5: 0, d6: 0, q6: q6,
                    alpha6: 0, a6: 0, d7: d7, q7: 0}

        def DH_Transform(alpha, a, d, theta):
            return Matrix(
                [[cos(theta), -sin(theta), 0, a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha) * d],
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha) * d],
                [0, 0, 0, 1]])

        t0_1 = DH_Transform(alpha0, a0, d1, q1).subs(dh_table)
        t1_2 = DH_Transform(alpha1, a1, d2, q2).subs(dh_table)
        t2_3 = DH_Transform(alpha2, a2, d3, q3).subs(dh_table)
        t3_4 = DH_Transform(alpha3, a3, d4, q4).subs(dh_table)
        t4_5 = DH_Transform(alpha4, a4, d5, q5).subs(dh_table)
        t5_6 = DH_Transform(alpha5, a5, d6, q6).subs(dh_table)
        t6_7 = DH_Transform(alpha6, a6, d7, q7).subs(dh_table)

        # Step 2: find Wrist Centre
        t0_ee = t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_7

        r, p, y = symbols('r p y')
        rotation_x = Matrix([
            [1, 0, 0],
            [0, cos(r), -sin(r)],
            [0, sin(r), cos(r)]])
        rotation_y = Matrix([
            [cos(p), 0, sin(p)],
            [0, 1, 0],
            [-sin(p), 0, cos(p)]])
        rotation_z = Matrix([
            [cos(y), -sin(y), 0],
            [sin(y), cos(y), 0],
            [0, 0, 1]])

        rotation_intrinsic = rotation_z * rotation_y * rotation_x
        rotation_correction = rotation_z.subs(y, pi) * rotation_y.subs(p, -pi/2)
    	#
    	#
    	# Create Modified DH parameters
    	#
    	#
    	# Define Modified DH Transformation matrix
    	#
    	#
    	# Create individual transformation matrices
    	#
    	#
    	# Extract rotation matrices from the transformation matrices
    	#
    	#
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            rotation_ee = (rotation_intrinsic * rotation_correction).subs({'r': roll, 'p': pitch, 'y': yaw})
            end_effector = Matrix([[px], [py], [pz]])

            ### Your IK code here
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            # translate along z-axis (See lesson 11.18 Inverse Kinematics)
            wrist_centre = end_effector - d7 * rotation_ee[:,2]

            # print 'Calculated WC is {}, test case centre is {}'.format(wrist_centre, test_case[1])

            # Step 3: find first 3 join angles

            theta1 = atan2(wrist_centre[1], wrist_centre[0])
            # print 'theta1 {} type {}'.format(theta1, type(theta1))

            # SSS and law of cosines
            # side c is simple
            side_c = a2
            # print 'side C {}'.format(side_c)
            # determine fixed length from joint 3 to joint 5
            side_a = sqrt(d4 ** 2 + a3 ** 2)
            # print 'side A {}'.format(side_a)
            # determine length from joint 2 to wrist centre
            side_b = sqrt(
                (sqrt(wrist_centre[0] ** 2 + wrist_centre[1] ** 2) - a1) ** 2 +
                (wrist_centre[2] - d1) ** 2)
            # print 'side B {}'.format(side_b)

            angle_a = acos((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))
            # print 'angle_a {}'.format(angle_a)
            angle_b = acos((side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c))
            # angle_c = pi - angle_a - angle_b
            angle_c = acos((side_a ** 2 + side_b ** 2 - side_c ** 2) / (2 * side_a * side_b))
            # print 'theta2 sqrt component {}'.format(sqrt(wrist_centre[0] ** 2 + wrist_centre[1] ** 2) - a1)
            # print 'theta2 arctan component {}'.format(atan2(wrist_centre[2] - d1, sqrt(wrist_centre[0] ** 2 + wrist_centre[1] ** 2) - a1))

            theta2 = pi / 2 - angle_a - atan2(wrist_centre[2] - d1, sqrt(wrist_centre[0] ** 2 + wrist_centre[1] ** 2) - a1)
            # print 'theta2 {} type {}'.format(theta2, type(theta2))

            side_d = sqrt(0.96 ** 2 + 0.054 ** 2)
            side_e = 0.54
            theta3_correction  =  atan2(abs(a3), d4)
            # print "theta3 correction {}".format(theta3_correction)
            theta3 = pi / 2 - angle_b - theta3_correction
            # print 'theta3 {} type {}'.format(theta3, type(theta3))

            R0_3 = t0_1[0:3, 0:3] * t1_2[0:3, 0:3] * t2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.transpose() * rotation_ee

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            # print 'theta4 {} type {}'.format(theta4, type(theta4))
            theta5 = atan2(sqrt(R3_6[0,2] ** 2 + R3_6[2,2] ** 2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    	    #
    	    #
    	    # Calculate joint angles using Geometric IK method
    	    #
    	    #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
