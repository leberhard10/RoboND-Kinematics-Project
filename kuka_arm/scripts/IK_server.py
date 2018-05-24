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

        # Create symbols for joint variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        s = {alpha0:   0.0, a0: 0.0,   d1: 0.75,
             alpha1: -pi/2, a1: 0.35,  d2: 0.0,  
             alpha2:   0.0, a2: 1.25,  d3: 0.0,
             alpha3: -pi/2, a3: -0.54, d4: 1.50,
             alpha4:  pi/2, a4: 0.0,   d5: 0.0,
             alpha5: -pi/2, a5: 0.0,   d6: 0.0,
             alpha6:   0.0, a6: 0.0,   d7: 0.303}
             
        # Define Modified DH Transformation matrix
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)
        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)
        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)
        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)        
        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)
        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)
        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
        T6_G = T6_G.subs(s)

        
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
        
	    # Extract rotation matrices from the transformation matrices
        r, p, y = symbols('r p y')
        Rot_x = Matrix([[ 1,              0,        0],
                      [ 0,        cos(r), -sin(r)],
                      [ 0,        sin(r),  cos(r)]])

        Rot_y = Matrix([[ cos(p),        0,  sin(p)],
                      [       0,        1,        0],
                      [-sin(p),        0,  cos(p)]])

        Rot_z = Matrix([[ cos(y), -sin(y),        0],
                      [ sin(y),  cos(y),        0],
                      [ 0,              0,        1]])
        
        Rot_EE = Rot_x * Rot_y * Rot_z
        Rot_error = Rot_z.subs(y, radians(180)) * Rot_y.subs(p, radians(-90))
        
        Rot_EE = Rot_EE * Rot_error
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

        ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
        Rot_EE = Rot_EE.subs({'r': roll, 'p':pitch, 'y':yaw})
        
        EE = Matrix([[px], [py], [px]])
        WC = EE -  0.360 * Rot_EE[:,2]     
                      
	    # Calculate joint angles using Geometric IK method
        theta1 = atan2(WC[1], WC[0])    
        print ("\nTheta 1", theta1)
        
        A = 1.50 #d40
        B = sqrt(pow(WC[1], 2) + pow(WC[0], 2))
        C = 1.25 #a2

        a = acos((pow(B,2) + pow(C,2) - pow(A,2))/(2*B*C))

        WCa = acos((pow(WC[1],2) + pow(B,2) - pow(WC[0],2))/(2*WC[1]*B))

        theta2 = pi/2 - a - WCa    
        print ("Theta 2", theta2)

        b = acos((pow(A,2) + pow(C,2) - pow(B,2))/(2*A*C))

        theta3 = pi/2 - b    
        print ("Theta 3", theta3)
        
        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        
        print ("\nTotal run time to eval R0_3 is %04.4f seconds" % (time()-start_time))
        
        R3_6 = R0_3.inv("LU") * Rot_EE
        
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])    
        print ("\nTheta 4", theta4)
        theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])    
        print ("\nTheta 5", theta5)
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])        
        print ("\nTheta 6", theta6)
        
        print ("\nTotal run time to thetas is %04.4f seconds" % (time()-start_time))


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
