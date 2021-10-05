#!/usr/bin/python

import rospy
from math import sin,cos
import numpy as np
import threading
from gazebo_msgs.msg import ModelStates
from scipy.optimize import minimize,fsolve
from geometry_msgs.msg import Twist
from px4_mavros import Px4Controller

P1,P2,P3,Pc,Pr,Pb,thetac = None,None,None,None,None,None,None
camera_cmd_vel = Twist()
ranging_cmd_vel = Twist()
bearing_cmd_vel = Twist()
fx,fy = 0.1496483333,0.1496483333
gamma = 1.0

def object_fun(x):
	return -( ((P1[0] - (Pb[0] + x[6]))*(P1[0] - (Pr[0] + x[3])) + (P1[1] - (Pb[1] + x[7]))*(P1[1] - (Pr[1] + x[4])))**2/(((P1[0] - (Pb[0] + x[6]))**2 + (P1[1] - (Pb[1] + x[7]))**2)**2*((P1[0] - (Pr[0] + x[3]))**2 + (P1[1] - (Pr[1] + x[4]))**2)) + (fx**2*((P1[0] - (Pr[0] + x[3]))*(P1[0] - (Pc[0] + x[0])) + (P1[1] - (Pr[1] + x[4]))*(P1[1] - (Pc[1] + x[1])))**2 + fy**2*(P1[2] - (Pc[2] + x[2]))**2*((P1[0] - (Pr[0] + x[3]))*sin(thetac + x[9]) - (P1[1] - (Pr[1] + x[4]))*cos(thetac + x[9]))**2)/((cos(thetac + x[9])*(P1[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P1[1] - (Pc[1] + x[1])))**4*((P1[0] - (Pr[0] + x[3]))**2 + (P1[1] - (Pr[1] + x[4]))**2)) + (fx**2*((P1[1] - (Pb[1] + x[7]))*(P1[0] - (Pc[0] + x[0])) + (P1[0] - (Pb[0] + x[6]))*(P1[1] - (Pc[1] + x[1])))**2 + fy**2*(P1[2] - (Pc[2] + x[2]))**2*((P1[1] - (Pb[1] + x[7]))*sin(thetac + x[9]) - (P1[0] - (Pb[0] + x[6]))*cos(thetac + x[9]))**2)/((cos(thetac + x[9])*(P1[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P1[1] - (Pc[1] + x[1])))**4*((P1[0] - (Pb[0] + x[6]))**2 + (P1[1] - (Pb[1] + x[7]))**2)**2) + fx**2*fy**2*(P1[2] - (Pc[2] + x[2]))**2/(cos(thetac + x[9])*(P1[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P1[1] - (Pc[1] + x[1])))**6 )*( ((P2[0] - (Pb[0] + x[6]))*(P2[0] - (Pr[0] + x[3])) + (P2[1] - (Pb[1] + x[7]))*(P2[1] - (Pr[1] + x[4])))**2/(((P2[0] - (Pb[0] + x[6]))**2 + (P2[1] - (Pb[1] + x[7]))**2)**2*((P2[0] - (Pr[0] + x[3]))**2 + (P2[1] - (Pr[1] + x[4]))**2)) + (fx**2*((P2[0] - (Pr[0] + x[3]))*(P2[0] - (Pc[0] + x[0])) + (P2[1] - (Pr[1] + x[4]))*(P2[1] - (Pc[1] + x[1])))**2 + fy**2*(P2[2] - (Pc[2] + x[2]))**2*((P2[0] - (Pr[0] + x[3]))*sin(thetac + x[9]) - (P2[1] - (Pr[1] + x[4]))*cos(thetac + x[9]))**2)/((cos(thetac + x[9])*(P2[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P2[1] - (Pc[1] + x[1])))**4*((P2[0] - (Pr[0] + x[3]))**2 + (P2[1] - (Pr[1] + x[4]))**2)) + (fx**2*((P2[1] - (Pb[1] + x[7]))*(P2[0] - (Pc[0] + x[0])) + (P2[0] - (Pb[0] + x[6]))*(P2[1] - (Pc[1] + x[1])))**2 + fy**2*(P2[2] - (Pc[2] + x[2]))**2*((P2[1] - (Pb[1] + x[7]))*sin(thetac + x[9]) - (P2[0] - (Pb[0] + x[6]))*cos(thetac + x[9]))**2)/((cos(thetac + x[9])*(P2[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P2[1] - (Pc[1] + x[1])))**4*((P2[0] - (Pb[0] + x[6]))**2 + (P2[1] - (Pb[1] + x[7]))**2)**2) + fx**2*fy**2*(P2[2] - (Pc[2] + x[2]))**2/(cos(thetac + x[9])*(P2[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P2[1] - (Pc[1] + x[1])))**6 )*( ((P3[0] - (Pb[0] + x[6]))*(P3[0] - (Pr[0] + x[3])) + (P3[1] - (Pb[1] + x[7]))*(P3[1] - (Pr[1] + x[4])))**2/(((P3[0] - (Pb[0] + x[6]))**2 + (P3[1] - (Pb[1] + x[7]))**2)**2*((P3[0] - (Pr[0] + x[3]))**2 + (P3[1] - (Pr[1] + x[4]))**2)) + (fx**2*((P3[0] - (Pr[0] + x[3]))*(P3[0] - (Pc[0] + x[0])) + (P3[1] - (Pr[1] + x[4]))*(P3[1] - (Pc[1] + x[1])))**2 + fy**2*(P3[2] - (Pc[2] + x[2]))**2*((P3[0] - (Pr[0] + x[3]))*sin(thetac + x[9]) - (P3[1] - (Pr[1] + x[4]))*cos(thetac + x[9]))**2)/((cos(thetac + x[9])*(P3[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P3[1] - (Pc[1] + x[1])))**4*((P3[0] - (Pr[0] + x[3]))**2 + (P3[1] - (Pr[1] + x[4]))**2)) + (fx**2*((P3[1] - (Pb[1] + x[7]))*(P3[0] - (Pc[0] + x[0])) + (P3[0] - (Pb[0] + x[6]))*(P3[1] - (Pc[1] + [1])))**2 + fy**2*(P3[2] - (Pc[2] + x[2]))**2*((P3[1] - (Pb[1] + x[7]))*sin(thetac + x[9]) - (P3[0] - (Pb[0] + x[6]))*cos(thetac + x[9]))**2)/((cos(thetac + x[9])*(P3[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P3[1] - (Pc[1] + x[1])))**4*((P3[0] - (Pb[0] + x[6]))**2 + (P3[1] - (Pb[1] + x[7]))**2)**2) + fx**2*fy**2*(P3[2] - (Pc[2] + x[2]))**2/(cos(thetac + x[9])*(P3[0] - (Pc[0] + x[0])) + sin(thetac + x[9])*(P3[1] - (Pc[1] + x[1])))**6 )
'''
def cons_maker(i=0):
	def constraint(x):
		return A[i,0]*x[0] + A[i,1]*x[1] + A[i,2]*x[2] + b[i] - x[i+3]
	return constraint

def cons_maker1(i=0):
	def constraint(x):
		return x[i+3]
	return constraint
'''
def odom(msg):
	global P1,P2,P3,Pc,Pr,Pb
	UAV1_index = msg.name.index('iris_camera')
	UAV2_index = msg.name.index('iris_ranging')
	UAV3_index = msg.name.index('iris_bearing')
	car1_index = msg.name.index('car1')
	car2_index = msg.name.index('car2')
	car3_index = msg.name.index('car3')

	Pc = np.array([msg.pose[UAV1_index].position.x, msg.pose[UAV1_index].position.y, msg.pose[UAV1_index].position.z])
	Pr = np.array([msg.pose[UAV2_index].position.x, msg.pose[UAV2_index].position.y, msg.pose[UAV2_index].position.z])
	Pb = np.array([msg.pose[UAV3_index].position.x, msg.pose[UAV3_index].position.y, msg.pose[UAV3_index].position.z])
	P1 = np.array([msg.pose[car1_index].position.x, msg.pose[car1_index].position.y, msg.pose[car1_index].position.z])
	P2 = np.array([msg.pose[car2_index].position.x, msg.pose[car2_index].position.y, msg.pose[car2_index].position.z])
	P3 = np.array([msg.pose[car3_index].position.x, msg.pose[car3_index].position.y, msg.pose[car3_index].position.z])

def	qpsolver():
	global camera_cmd_vel,ranging_cmd_vel,bearing_cmd_vel
	
	cons = []
	'''
	for i in range (b.size):
		cons.append({'type': 'eq', 'fun': cons_maker(i)})
	for i in range (b.size):
		cons.append({'type': 'ineq', 'fun': cons_maker1(i)})
	'''
	ini = tuple(np.zeros(10))
	bnds = ((-1.5, 1.5), (-1.5, 1.5), (-1.0, 1.0), (-1.5, 1.5), (-1.5, 1.5), (-1.0, 1.0), (-1.5, 1.5), (-1.5, 1.5), (-1.0, 1.0), (-0.2, 0.2))
	
	optimal = minimize(object_fun, ini, method='SLSQP', bounds=bnds, constraints=cons,options={'maxiter':1000}).x
	print(optimal)
	camera_cmd_vel.linear.x = optimal[0]
	camera_cmd_vel.linear.y = optimal[1]
	camera_cmd_vel.linear.z = optimal[2]
	ranging_cmd_vel.linear.x = optimal[3]
	ranging_cmd_vel.linear.y = optimal[4]
	ranging_cmd_vel.linear.z = optimal[5]
	bearing_cmd_vel.linear.x = optimal[6]
	bearing_cmd_vel.linear.y = optimal[7]
	bearing_cmd_vel.linear.z = optimal[8]
	camera_cmd_vel.angular.z = optimal[9]
	
	px4_camera.vel_control(camera_cmd_vel)
	px4_ranging.vel_control(ranging_cmd_vel)
	px4_bearing.vel_control(bearing_cmd_vel)

if __name__ == '__main__':
	try:
		rospy.init_node('controller')
		uavtype = ["iris_camera","iris_ranging","iris_bearing"]
		px4_camera = Px4Controller(uavtype[0])
		px4_ranging = Px4Controller(uavtype[1])
		px4_bearing = Px4Controller(uavtype[2])
		rate = rospy.Rate(20)
		
		while not rospy.is_shutdown():
			msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)
			while thetac == None:
				thetac = px4_camera.current_heading
			odom(msg)
			qpsolver()
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
