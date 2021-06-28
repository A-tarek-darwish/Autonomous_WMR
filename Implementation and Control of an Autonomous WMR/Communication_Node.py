#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
#from geometry_msgs.msg import Twist
import roslib
import math
#from rospy import init_node, is_shutdown
import numpy
import time


def sendpos(x):
        rospy.init_node('communication', anonymous=True)
	pub = rospy.Publisher("/ygoal", numpy_msg(Int32), queue_size=10)
	rospy.init_node('sender', anonymous = True)
	rate = rospy.Rate(1)
	msg = x
	while pub.get_num_connections() < 1:
            #rospy.loginfo(msg)
            pub.publish(msg)

def getvelocity():
        #global position
        #rospy.init_node('planning', anonymous=True)
        #rospy.Subscriber('/velocity', Int32, callback)
        y= rospy.wait_for_message("/velocity", Int32, timeout=None)
        return y

x_Goal = int(raw_input("Enter the goal value of X :"))
y_Goal = int(raw_input("Enter the goal value of Y :"))

position= np.array([x_Goal, y_Goal])
sendpos(position)
while true:
	v=getvelocity()
	ser.write(b'v')
	time.sleep(1)
