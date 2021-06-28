#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String 
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
import roslib
import serial
import time




a=10
b=10
angle=0 

x_mat = np.zeros(a)
y_mat = np.zeros(b)
i =0
curr_x = x_mat[i]
curr_y = y_mat[i]
goal_x =0
goal_y =0


#Method to receive x-array

def getx_array():
	global x_mat   
        rospy.init_node('receive_xpath', anonymous=True)
        #rospy.Subscriber("/xpath", numpy_msg(Int32), callback)
        x_mat= rospy.wait_for_message("/xpath", numpy_msg(Int32), timeout=None)
        return x_mat

getx_array()

#Method to receive y-array

def gety_array():   
	global y_mat
        rospy.init_node('receive_ypath', anonymous=True)
        #rospy.Subscriber("/ypath", numpy_msg(Int32), callback)
        y_mat= rospy.wait_for_message("/ypath", numpy_msg(Int32), timeout=None)
        return y_mat

gety_array()

#Method to receive x-goal

def getx_goal():   
	global goal_x
        rospy.init_node('receive_xgoal', anonymous=True)
        #rospy.Subscriber("/xgoal", Int32, callback)
        goal_x= rospy.wait_for_message("/xgoal", Int32, timeout=None)
        return goal_x

getx_goal()

#Method to receive y-goal

def gety_goal():   
	global goal_y
        rospy.init_node('receive_ygoal', anonymous=True)
        #rospy.Subscriber("/ygoal", Int32, callback)
        goal_y= rospy.wait_for_message("/ygoal", Int32, timeout=None)
        return goal_y

gety_goal()

#Method to decide the motion of the robot
def defineMotion():
	ser = serial.Serial('/dev/ttyACM0', 9600, timeout=None)
    	ser.flush()
	global curr_x , curr_y , goal_x , goal_y
	while curr_x != goal_x and curr_y != goal_y :
		if x_mat[i+1] == (curr_x + 1) and y_mat[i+1] == curr_y :
		        if angle == -45:
		            ser.write(b"2")
		            angle = -90
		        if angle == -180 or angle ==180:
		            ser.write(b"1")
		            angle = -135
		        if angle == 0:
		            ser.write(b"1")
		            angle = 45
		        if angle == 45:
		            ser.write(b"1")
		            angle = 90
		        if angle == -135:
		            ser.write(b"1")
		            angle = -90
		        if angle == 135:
		            ser.write(b"2")
		            angle = 90
		        if angle == -90:
		            ser.write(b"4")
		            i = i+1
		        if angle == 90:
		            ser.write(b"3")
		            i = i+1
		
		if x_mat[i+1] == (curr_x - 1) and y_mat[i+1] == curr_y :
		        if angle == -45:
		            ser.write(b"2")
		            angle = -90
		        if angle == -180 or angle == 180:
		            ser.write(b"1")
		            angle = -135
		        if angle == 0:
		            ser.write(b"1")
		            angle = 45
		        if angle == 45:
		            ser.write(b"1")
		            angle = 90
		        if angle == -135:
		            ser.write(b"1")
		            angle = -90
		        if angle == 135:
		            ser.write(b"2")
		            angle = 90
		        if angle == -90:
		            ser.write(b"3")
		            i = i+1
		        if angle == 90:
		            ser.write(b"4")
		            i = i+1

		if x_mat[i+1] == curr_x and y_mat[i+1] == curr_y + 1 :
		        if angle == 135:
		            ser.write(b"1")
		            angle = 180
		        if angle == -135:
		            ser.write(b"2")
		            angle = -180
		        if angle == 90:
		            ser.write(b"2")
		            angle = 45
		        if angle == -90:
		            ser.write(b"1")
		            angle = -45
		        if angle == -45:
		            ser.write(b"1")
		            angle = 0
		        if angle == 45:
		            ser.write(b"2")
		            angle = 0
		        if angle == 0:
		            ser.write(b"3")
		            i = i+1
		        if angle == -180 or angle == 180:
		            ser.write(b"4")
		            i = i + 1
		        
		if x_mat[i+1] == curr_x and y_mat[i+1] == curr_y - 1 :
		        if angle == 135:
		            ser.write(b"1")
		            angle = 180
		        if angle == -135:
		            ser.write(b"2")
		            angle = -180
		        if angle == 90:
		            ser.write(b"2")
		            angle = 45
		        if angle == -90:
		            ser.write(b"1")
		            angle = -45
		        if angle == -45:
		            ser.write(b"1")
		            angle = 0
		        if angle == 45:
		            ser.write(b"2")
		            angle = 0
		        if angle == 0:
		            ser.write(b"4")
		            i = i+1
		        if angle == -180 or angle == 180:
		            ser.write(b"3")
		            i = i+1

		if x_mat[i+1] == curr_x + 1 and y_mat[i+1] == curr_y + 1 :
		        if angle == 135:
		            ser.write(b"2")
		            angle = 90
		        if angle == 90:
		            ser.write(b"2")
		            angle = 45
		        if angle == -45:
		            ser.write(b"2")
		            angle = -90
		        if angle == -90:
		            ser.write(b"2")
		            angle = -135
		        if angle == 0:
		            ser.write(b"1")
		            angle =45
		        if angle == -180 or angle == 180:
		            ser.write(b"1")
		            angle = -135
		        if angle == -135:
		            ser.write(b"6")
		            i = i+1
		        if angle == 45:
		            ser.write(b"5")
		            i = i+1
		    
		if x_mat[i+1] == curr_x - 1 and y_mat[i+1] == curr_y - 1 :
		        if angle == 135:
		            ser.write(b"2")
		            angle = 90
		        if angle == 90:
		            ser.write(b"2")
		            angle = 45
		        if angle == -45:
		            ser.write(b"2")
		            angle = -90
		        if angle == -90:
		            ser.write(b"2")
		            angle = -135
		        if angle == 0:
		            ser.write(b"1")
		            angle =45
		        if angle == -180 or angle == 180:
		            ser.write(b"1")
		            angle = -135
		        if angle == -135:
		            ser.write(b"5")
		            i = i+1
		        if angle == 45:
		            ser.write(b"6")
		            i = i+1

		if x_mat[i+1] == curr_x - 1 and y_mat[i+1] == curr_y + 1 :
		        if angle == 0:
		            ser.write(b"2")
		            angle = -45
		        if angle == -180 or angle == 180:
		            ser.write(b"2")
		            angle = 135
		        if angle == -135:
		            ser.write(b"1")
		            angle = -90
		        if angle == 45:
		            ser.write(b"1")
		            angle = 90
		        if angle == 90:
		            ser.write(b"1")
		            angle = 135
		        if angle == -90:
		            ser.write(b"1")
		            angle = -45
		        if angle == 135:
		            ser.write(b"6")
		            i = i+1
		        if angle == -45:
		            ser.write(b"5")
		            i = i+1

		if x_mat[i+1] == curr_x + 1 and y_mat[i+1] == curr_y - 1 :
		        if angle == 0:
		            ser.write(b"2")
		            angle = -45
		        if angle == -180 or angle == 180:
		            ser.write(b"2")
		            angle = 135
		        if angle == -135:
		            ser.write(b"1")
		            angle = -90
		        if angle == 45:
		            ser.write(b"1")
		            angle = 90
		        if angle == 90:
		            ser.write(b"1")
		            angle = 135
		        if angle == -90:
		            ser.write(b"1")
		            angle = -45
		        if angle == 135:
		            ser.write(b"5")
		            i = i+1
		        if angle == -45:
		            ser.write(b"6")
		            i = i+1

defineMotion()
