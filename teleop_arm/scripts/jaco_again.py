#!/usr/bin/env python3

import rospy
import numpy as np
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Log
import re

# Global variables for joystick velocities
X, Y, Z = 0.0, 0.0, 0.0  # Initialize velocities as 0
joint_angles = np.zeros(3)

def Rz(x):
    return np.array([[math.cos(x), -math.sin(x), 0],
                     [math.sin(x),  math.cos(x), 0],
                     [0, 0, 1]])

def Ry(x):    
    return np.array([[math.cos(x), 0, math.sin(x)],
                     [0, 1, 0],
                     [-math.sin(x), 0, math.cos(x)]])

# ROS Callback for /rosout to extract joystick velocities
def rosout_callback(data):
    global X, Y, Z
    """
    Callback function to process /rosout messages and extract x, y, z velocities.
    """
    # Extracting message string from /rosout
    log_msg = data.msg
    
    # Use regex to extract joystick data from the log message
    match = re.search(r'"x":\s*([-\d.e]+),\s*"y":\s*([-\d.e]+),\s*"z":\s*([-\d.e]+)', log_msg)
    
    if match:
        try:
            # Parse the extracted x, y, z values
            X = float(match.group(1))*0.2
            Y = float(match.group(2))*0.2
            Z = float(match.group(3))*0.2
            rospy.loginfo("Publishing joystick velocities - X: {}, Y: {}, Z: {}".format(X, Y, Z))
            
        except ValueError as e:
            rospy.logerr("Error converting joystick values: {}".format(e))

# Function to compute transformation matrix and Jacobian inverse
def get_matrix(joint_angles):
    A = 0.1745
    B = 0.435
    C = 0.335 + 0.060
    a, b, c = joint_angles
    
    # Homogeneous transformations
    R_0_1 = np.dot(Rz(a), np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]))
    d_0_1 = np.array([0, 0, A])
    
    R_1_2 = np.dot(Rz(b), np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]]))
    d_1_2 = np.array([B * math.sin(b), -B * math.cos(b), 0])
    
    R_2_3 = np.dot(Rz(c), np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]))
    d_2_3 = np.array([C * math.sin(c), -C * math.cos(c), 0])
    
    R_0_2 = R_0_1 @ R_1_2
    R_0_3 = R_0_2 @ R_2_3
    
    H_0_1 = np.vstack((np.hstack((R_0_1, d_0_1.reshape(3, 1))), np.array([0, 0, 0, 1])))
    H_1_2 = np.vstack((np.hstack((R_1_2, d_1_2.reshape(3, 1))), np.array([0, 0, 0, 1])))
    H_2_3 = np.vstack((np.hstack((R_2_3, d_2_3.reshape(3, 1))), np.array([0, 0, 0, 1])))

    H_0_2 = H_0_1 @ H_1_2
    H_0_3 = H_0_1 @ H_1_2 @ H_2_3
    
    d_0_2 = H_0_2[:3, 3]
    d_0_3 = H_0_3[:3, 3]
    
    # Jacobian columns (cross products)
    col1 = np.cross([0, 0, 1], d_0_3)
    col2 = np.cross(R_0_1 @ [0, 0, 1], d_0_3 - d_0_1)
    col3 = np.cross(R_0_2 @ [0, 0, 1], d_0_3 - d_0_2)

    # Construct Jacobian
    jaco = np.column_stack((col1, col2, col3))
    
    # Invert the Jacobian (use pseudo-inverse to avoid singularities)
    jaco_inv = np.linalg.inv(jaco)
    
    return jaco_inv

# Function to compute joint velocities using the inverse Jacobian
def compute_joint_velocities(X, Y, Z, joint_angles):
    J = get_matrix(joint_angles)
    end_effector_velocities = np.array([X, Y, Z])
    joint_velocities = J.dot(end_effector_velocities)
    return joint_velocities

# Function to publish joint velocities
def publish_joint_velocities(pub, joint_velocities):
    global joint_angles
    
    msg = Float64MultiArray()
    angles = joint_angles + joint_velocities * 0.1  # Update joint angles
    msg.data = [angles[0], angles[1], angles[2], 0]

    rospy.loginfo(f"Publishing joint velocities: {joint_velocities}")
    pub.publish(msg)

# Subscriber to get current joint states
def joint_sub(msg: JointState):
    global joint_angles
    joint_angles = np.array(msg.position[:3])

def main():
    global X, Y, Z, joint_angles

    rospy.init_node('teleop_arm')
    pub = rospy.Publisher('/plan_controller/command', Float64MultiArray, queue_size=10)
    sub = rospy.Subscriber('/joint_states', JointState, joint_sub)
    
    # Subscribe to /rosout to get joystick velocities
    rospy.Subscriber('/rosout', Log, rosout_callback)
    
    rate = rospy.Rate(100)  # 100 Hz

    joint_angles = np.array([0.0, 0.0, 0.0])  # Initial joint angles

    rospy.loginfo("Teleoperation node started. Use joystick to control the arm.")

    while not rospy.is_shutdown():
        # Compute joint velocities from the joystick velocities
        joint_velocities = compute_joint_velocities(X, Y, Z, joint_angles)

        # Publish the joint velocities
        publish_joint_velocities(pub, joint_velocities)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
