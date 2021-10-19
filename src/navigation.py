#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

Pc,Pr,Pb = None,None,None
l_Pc,l_Pr,l_Pb = np.array([0,0,0]),np.array([0,0,0]),np.array([0,0,0])
err_c,err_r,err_b = np.inf,np.inf,np.inf
car1_cmd_vel,car2_cmd_vel,car3_cmd_vel = Twist(),Twist(),Twist()
time = 0

def odom(msg):
	global Pc,Pr,Pb,l_Pc,l_Pr,l_Pb,err_c,err_r,err_b
	UAV1_index = msg.name.index('iris_camera')
	UAV2_index = msg.name.index('iris_ranging')
	UAV3_index = msg.name.index('iris_bearing')

	Pc = np.array([msg.pose[UAV1_index].position.x, msg.pose[UAV1_index].position.y, msg.pose[UAV1_index].position.z])
	Pr = np.array([msg.pose[UAV2_index].position.x, msg.pose[UAV2_index].position.y, msg.pose[UAV2_index].position.z])
	Pb = np.array([msg.pose[UAV3_index].position.x, msg.pose[UAV3_index].position.y, msg.pose[UAV3_index].position.z])
	err_c = np.linalg.norm(Pc - l_Pc)
	err_r = np.linalg.norm(Pr - l_Pr)
	err_b = np.linalg.norm(Pb - l_Pb)
	l_Pc = Pc
	l_Pr = Pr
	l_Pb = Pb

def start():
	global car1_cmd_vel,car2_cmd_vel,car3_cmd_vel
	car1_cmd_vel.linear.x = 0.2
	car2_cmd_vel.linear.x = 0.2
	car3_cmd_vel.linear.x = 0.2
	car1_vel_pub.publish(car1_cmd_vel)
	car2_vel_pub.publish(car2_cmd_vel)
	car3_vel_pub.publish(car3_cmd_vel)

def control():
	global car1_cmd_vel,car2_cmd_vel,car3_cmd_vel,time
	'''	
	if time < 101:
		car1_cmd_vel.linear.x = 0.2
		car2_cmd_vel.linear.x = 0.2
		car3_cmd_vel.linear.x = 0.2
	elif time < 161:
		car1_cmd_vel.linear.x = 0.2
		car2_cmd_vel.linear.x = 0.2
		car3_cmd_vel.linear.x = 0.2
		car1_cmd_vel.angular.z = -0.2
		car2_cmd_vel.angular.z = -0.2
		car3_cmd_vel.angular.z = -0.2
	elif time < 261:
		car1_cmd_vel.linear.x = 0.2
		car2_cmd_vel.linear.x = 0.2
		car3_cmd_vel.linear.x = 0.2
	elif time < 321:
		car1_cmd_vel.linear.x = 0.2
		car2_cmd_vel.linear.x = 0.2
		car3_cmd_vel.linear.x = 0.2
		car1_cmd_vel.angular.z = 0.2
		car2_cmd_vel.angular.z = 0.2
		car3_cmd_vel.angular.z = 0.2
	elif time < 421:
		car1_cmd_vel.linear.x = 0.2
		car2_cmd_vel.linear.x = 0.2
		car3_cmd_vel.linear.x = 0.2
	'''
	if time < 600:
		car1_cmd_vel.linear.x = 0.2
		car2_cmd_vel.linear.x = 0.2
		car3_cmd_vel.linear.x = 0.2

	car1_vel_pub.publish(car1_cmd_vel)
	car2_vel_pub.publish(car2_cmd_vel)
	car3_vel_pub.publish(car3_cmd_vel)
	time = time+1

def stop():
	global car1_cmd_vel,car2_cmd_vel,car3_cmd_vel
	car1_cmd_vel.linear.x = 0.0
	car2_cmd_vel.linear.x = 0.0
	car3_cmd_vel.linear.x = 0.0
	car1_cmd_vel.angular.z = 0.0
	car2_cmd_vel.angular.z = 0.0
	car3_cmd_vel.angular.z = 0.0
	car1_vel_pub.publish(car1_cmd_vel)
	car2_vel_pub.publish(car2_cmd_vel)
	car3_vel_pub.publish(car3_cmd_vel)

if __name__ == '__main__':
	try:
		rospy.init_node('navigation')
		car1_vel_pub = rospy.Publisher("/car1/cmd_vel",Twist,queue_size=1)
		car2_vel_pub = rospy.Publisher("/car2/cmd_vel",Twist,queue_size=1)
		car3_vel_pub = rospy.Publisher("/car3/cmd_vel",Twist,queue_size=1)
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)
			odom(msg)
			if err_c < 0.001 and err_r < 0.001 and err_b < 0.001:
				start()
				time = 1
			if time > 0:
				control()
			if time > 600:
				stop()
				break
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
