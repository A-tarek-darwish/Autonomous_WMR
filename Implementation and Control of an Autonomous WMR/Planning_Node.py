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

position= numpy.array([0,0])


#def callback():
    #global position
   # position=data.data


def getpos():
        #global position
        #rospy.init_node('planning', anonymous=True)
        #rospy.Subscriber('/position', String, callback)
        y= rospy.wait_for_message("/position", numpy_msg(Int32), timeout=None)
        return y


        
def sendxpath(x):
	pub = rospy.Publisher("/xpath", numpy_msg(Int32), queue_size=10)
	rospy.init_node('sender', anonymous = True)
	rate = rospy.Rate(1)
	msg = x
	
	while pub.get_num_connections() < 1:
            #rospy.loginfo(msg)
            pub.publish(msg)

        
def sendxgoal(x):
	pub = rospy.Publisher("/xgoal", Int32, queue_size=10)
	rospy.init_node('sender', anonymous = True)
	rate = rospy.Rate(1)
	msg = x
	
	while pub.get_num_connections() < 1:
            #rospy.loginfo(msg)
            pub.publish(msg)

def sendygoal(x):
	pub = rospy.Publisher("/ygoal", Int32, queue_size=10)
	rospy.init_node('sender', anonymous = True)
	rate = rospy.Rate(1)
	msg = x
	
	while pub.get_num_connections() < 1:
            #rospy.loginfo(msg)
            pub.publish(msg)


def sendypath(x):
	pub = rospy.Publisher("/ypath", numpy_msg(Int32), queue_size=10)
	rospy.init_node('sender', anonymous = True)
	rate = rospy.Rate(1)
	msg = x
	
	while pub.get_num_connections() < 1:
            #rospy.loginfo(msg)
            pub.publish(msg)            
            
    
def Dis_to_Goal():
    global h, w, bw_img, x_Goal, y_Goal
    dx_mat = [0] * bw_img
    dy_mat = [0] * bw_img
    dis_mat = [0] * bw_img
    for i in range(0,h):
	for j in range(0,w): 
  	    dy_mat[i,j] = abs(y_Goal - i)
	    dx_mat[i,j] = abs(x_Goal - j)
    dis_mat = np.sqrt(np.square(dx_mat)+np.square(dy_mat))
    return dis_mat, dx_mat, dy_mat

def A_star():
    global x_Start, y_Start, x_Goal, y_Goal, dis_mat, h, w
    path_x = [x_Start]
    path_y = [y_Start]
    x_curr = x_Start
    y_curr = y_Start
    mask_x = np.array([[-1,0,1], [-1,0,1], [-1,0,1]])
    mask_y = np.array([[-1,-1,-1], [0,0,0], [1,1,1]])
    dis_to_goal_list = []
    dis_from_start_list = []
    x_neig_index = []
    y_neig_index = []
    Travelled_distance = [0]

    #for i in range(0,1):
    while any([x_curr != x_Goal,y_curr != y_Goal]) and not rospy.is_shutdown():
	if bw_img[y_curr,x_curr] != 0: 
	        ind_x = mask_x + [x_curr]
                ind_y = mask_y + [y_curr]
	        for k in range(0,3):
		    for l in range(0,3):
		        if all([ind_x[k,l] >= 0,ind_x[k,l] < w,ind_y[k,l] >= 0,ind_y[k,l] < h]):
			    l1 = np.where(path_x == ind_x[k,l])[0]
			    l2 = np.where(path_y == ind_y[k,l])[0]
			    flag = 0
			    for i in range(0,len(l1)):
			        for j in range(0,len(l2)):
				    if l1[i] == l2[j]:
				        flag = 1
			    if  any([all([ind_x[k,l] == x_curr,ind_y[k,l] == y_curr]),flag == 1]):
			        pass
  			    else:
				if bw_img[ind_y[k,l],ind_x[k,l]] != 0:
				    #if neighbour is already in the path
				    x_neig_index = np.concatenate((x_neig_index, mask_x[k,l]), axis=None)
				    y_neig_index = np.concatenate((y_neig_index, mask_y[k,l]), axis=None)
				    dis_to_goal_list = np.concatenate((dis_to_goal_list, dis_mat[ind_y[k,l],ind_x[k,l]]), axis=None)
				    dis_from_start_list = np.concatenate((dis_from_start_list, Travelled_distance+np.sqrt(np.square(mask_x[k,l])+np.square(mask_y[k,l]))), axis=None)
	        #print('x_neighbous', x_neig_index)
	        #print('y_neighbous', y_neig_index)
                #print('dis_to_goal_list', dis_to_goal_list)
	        #print('dis_from_start_list', dis_from_start_list)
	 
	obj_fn = dis_to_goal_list + dis_from_start_list
	obj_fn = dis_to_goal_list
	result = np.where(obj_fn == np.amin(obj_fn))
	path_x = np.concatenate((path_x, x_curr+x_neig_index[result[0][0]]), axis=None)
	path_y = np.concatenate((path_y, y_curr+y_neig_index[result[0][0]]), axis=None)
	Travelled_distance = dis_from_start_list[result[0][0]]
	x_curr = float(x_curr+x_neig_index[result[0][0]])
	y_curr = float(y_curr+y_neig_index[result[0][0]])
	x_neig_index = []
	y_neig_index = []
	dis_to_goal_list = []
        dis_from_start_list = []
	#print('path_x', path_x)
	#print('path_y', path_y)
	#print('obj_fn', obj_fn)
	#print('-----------------------------')
    return path_x, path_y



if __name__=='__main__':
    h=10
    w=10
    bw_img=255* np.ones(h,w)
    initialpos= getpos()
    x_start=getpos[0]
    y_start=getpos[1]
    x_Goal = int(raw_input("Enter the goal value of X :"))	   #recieve x_Goal input type casted as int
    y_Goal = int(raw_input("Enter the goal value of Y :"))
    #apply trajectory planning
    if x_Goal > w:
	print('Warning MSG: the goal value of X should not exceed', w, '. Please enter a new start value.')
	x_Goal = int(raw_input("Enter the goal value of X :"))	   #recieve x_Start input type casted as int
    if y_Goal > h:
	print('Warning MSG: the goal value of Y should not exceed', h, '. Please enter a new start value.')
	y_Goal = int(raw_input("Enter the goal value of Y :"))	   #recieve y_Start input type casted as int

    if x_Start == x_Goal and y_Start == y_Goal:
	print('Warning MSG: You entered the same start and goal points. Please choose 2 different set of points')
	x_Start = int(raw_input("Enter the start value of X :"))	   #recieve x_Start input type casted as int
	y_Start = int(raw_input("Enter the start value of Y :"))	   #recieve y_Start input type casted as int
	x_Goal = int(raw_input("Enter the goal value of X :"))	   #recieve x_Start input type casted as int
	y_Goal = int(raw_input("Enter the goal value of Y :"))	   #recieve y_Start input type casted as int

    if bw_img[y_Start,x_Start] == 0:
	print('Warning MSG: Sorry this start point is already occupied with an obstacle')
	x_Start = int(raw_input("Enter the start value of X :"))	   #recieve x_Start input type casted as int
	y_Start = int(raw_input("Enter the start value of Y :"))	   #recieve y_Start input type casted as int

    if bw_img[y_Goal,x_Goal] == 0:
	print('Warning MSG: Sorry this goal point is already occupied with an obstacle')
	x_Goal = int(raw_input("Enter the goal value of X :"))	   #recieve x_Start input type casted as int
	y_Goal = int(raw_input("Enter the goal value of Y :"))	   #recieve y_Start input type casted as int	
    
    dis_mat, dx_mat, dy_mat = Dis_to_Goal()

    
    #publish path to control
    path_x, path_y = A_star()
    
    
    sendxgoal(x_Goal)
    sendygoal(y_Goal)
    sendxpath(path_x)
    sendypath(path_y)
