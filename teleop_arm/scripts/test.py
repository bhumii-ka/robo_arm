#!/usr/bin/env python3

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math 

def Rz(x):
    return np.array([[math.cos(x),-math.sin(x),0],[math.sin(x),math.cos(x),0],[0,0,1]])

def Ry(x):    
    return np.array([[math.cos(x),0,math.sin(x)],[0,1,0],[-math.sin(x),0,math.cos(x)]])

def get_matrix(a, b, c):
    A=0.1745
    B = 0.435
    C = 0.335 + 0.060

    R_0_1=np.dot(Rz(a),np.array([[1,0,0],[0,0,1],[0,-1,0]]))
    d_0_1=np.array([0,0,A])
    R_1_2=np.dot(Rz(b),np.array([[0,-1,0],[-1,0,0],[0,0,-1]]))
    d_1_2=np.array([B*math.sin(b),-B*math.cos(b),0])
    R_2_3=np.dot(Rz(c),np.array([[0,1,0],[-1,0,0],[0,0,1]]))
    d_2_3=np.array([C*math.sin(c),-C*math.cos(c),0])
    R_0_2=R_0_1@R_1_2

    H_0_1 = np.vstack((np.hstack((R_0_1, d_0_1.reshape(3, 1))), np.array([0, 0, 0, 1])))
    H_1_2 = np.vstack((np.hstack((R_1_2, d_1_2.reshape(3, 1))), np.array([0, 0, 0, 1])))
    H_2_3 = np.vstack((np.hstack((R_2_3, d_2_3.reshape(3, 1))), np.array([0, 0, 0, 1])))
    H_0_2= H_0_1@H_1_2
    H_0_3= H_0_1@H_1_2@H_2_3
    d_0_2=np.array([H_0_2[0][3],H_0_2[1][3],H_0_2[2][3]])    
    d_0_3=np.array([H_0_3[0][3],H_0_3[1][3],H_0_3[2][3]])
    
    col1 = np.cross([0,0,1],d_0_3)
    col2= np.cross(R_0_1@[0,0,1],d_0_3-d_0_1)    
    col3= np.cross(R_0_2@[0,0,1],d_0_3-d_0_2)

    jaco = np.column_stack((col1, col2, col3))
    
    jaco_inv = np.linalg.inv(jaco)
    return jaco_inv

print(get_matrix(0,3.14,0)@np.array([0,1,0]))