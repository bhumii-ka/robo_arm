#!/usr/bin/env python3

import rospy
import re
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Log

def rosout_callback(data):
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
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))

            rospy.loginfo("Publishing joystick velocities - X: {}, Y: {}, Z: {}".format(x, y, z))
            
        except ValueError as e:
            rospy.logerr("Error converting joystick values: {}".format(e))
    
def joystick_listener():

    # Subscribe to /rosout topic to receive log messages
    rospy.Subscriber('/rosout', Log, rosout_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        joystick_listener()
    except rospy.ROSInterruptException:
        pass
