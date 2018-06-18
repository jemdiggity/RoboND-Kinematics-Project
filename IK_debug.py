from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##
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
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [req.poses[0].orientation.x,
        req.poses[0].orientation.y,
        req.poses[0].orientation.z,
        req.poses[0].orientation.w])

    rotation_ee = (rotation_intrinsic * rotation_correction).subs({'r': roll, 'p': pitch, 'y': yaw})

    px, py, pz = req.poses[0].position.x, req.poses[0].position.y, req.poses[0].position.z
    end_effector = Matrix([[px], [py], [pz]])

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
#    angle_c = pi - angle_a - angle_b
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

    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    FK = t0_ee.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wrist_centre[0], wrist_centre[1], wrist_centre[2]] # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3], FK[1,3], FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
