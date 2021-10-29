#!/usr/bin/env python

import rospy
from ukf import UKF
import numpy as np
from math import sin,cos,sqrt,atan2
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray

P1,P2,P3,Pc,Pr,Pb = None,None,None,None,None,None
fx,fy,Cu,Cv = 565.6,565.6,320.0,240.0
d_t = 0.05

# Process Noise
q = np.eye(12)
q[0][0] = 0.0001
q[1][1] = 0.0001
q[2][2] = 0.0025
q[3][3] = 0.0025
q[4][4] = 0.0001
q[5][5] = 0.0001
q[6][6] = 0.0025
q[7][7] = 0.0025
q[8][8] = 0.0001
q[9][9] = 0.0001
q[10][10] = 0.0025
q[11][11] = 0.0025

# create measurement noise covariance matrices
r_measurement = np.eye(12)
r_measurement[0][0] = 0.000049
r_measurement[1][1] = 0.000049
r_measurement[2][2] = 0.0001
r_measurement[3][3] = 0.0001
r_measurement[4][4] = 0.000049
r_measurement[5][5] = 0.000049
r_measurement[6][6] = 0.0001
r_measurement[7][7] = 0.0001
r_measurement[8][8] = 0.000049
r_measurement[9][9] = 0.000049
r_measurement[10][10] = 0.0001
r_measurement[11][11] = 0.0001

def iterate_x(x_in, timestep, inputs):
    '''this function is based on the x_dot and can be nonlinear as needed'''
    ret = np.zeros(len(x_in))
    ret[0] = x_in[0] + timestep * x_in[2]
    ret[1] = x_in[1] + timestep * x_in[3]
    ret[2] = x_in[2]
    ret[3] = x_in[3]
    ret[4] = x_in[4] + timestep * x_in[6]
    ret[5] = x_in[5] + timestep * x_in[7]
    ret[6] = x_in[6]
    ret[7] = x_in[7]
    ret[8] = x_in[8] + timestep * x_in[10]
    ret[9] = x_in[9] + timestep * x_in[11]
    ret[10] = x_in[10]
    ret[11] = x_in[11]
    return ret

def measurement_model(x_in, data):
    """
    :param x_in: states
    :param data: UAV positions of c,r,b and thetac
    """
    z1,z2,z3 = 0.0,0.0,0.0
    ret = np.zeros(len(x_in))
    ret[0] = fx*(sin(data[9])*(x_in[0] - data[0]) - cos(data[9])*(x_in[1] - data[1]))/(cos(data[9])*(x_in[0] - data[0]) + sin(data[9])*(x_in[1] - data[1])) + Cu
    ret[1] = -fy*(z1 - data[2])/(cos(data[9])*(x_in[0] - data[0]) + sin(data[9])*(x_in[1] - data[1])) + Cv
    ret[2] = sqrt((x_in[0] - data[3])**2 + (x_in[1] - data[4])**2)
    ret[3] = atan2((x_in[1] - data[7]),(x_in[0] - data[6]))
    ret[4] = fx*(sin(data[9])*(x_in[4] - data[0]) - cos(data[9])*(x_in[5] - data[1]))/(cos(data[9])*(x_in[4] - data[0]) + sin(data[9])*(x_in[5] - data[1])) + Cu
    ret[5] = -fy*(z2 - data[2])/(cos(data[9])*(x_in[4] - data[0]) + sin(data[9])*(x_in[5] - data[1])) + Cv
    ret[6] = sqrt((x_in[4] - data[3])**2 + (x_in[5] - data[4])**2)
    ret[7] = atan2((x_in[5] - data[7]),(x_in[4] - data[6]))
    ret[8] = fx*(sin(data[9])*(x_in[8] - data[0]) - cos(data[9])*(x_in[9] - data[1]))/(cos(data[9])*(x_in[8] - data[0]) + sin(data[9])*(x_in[9] - data[1])) + Cu
    ret[9] = -fy*(z3 - data[2])/(cos(data[9])*(x_in[8] - data[0]) + sin(data[9])*(x_in[9] - data[1])) + Cv
    ret[10] = sqrt((x_in[8] - data[3])**2 + (x_in[9] - data[4])**2)
    ret[11] = atan2((x_in[9] - data[7]),(x_in[8] - data[6]))
    return ret

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

def ukf():
    state_estimator.predict(d_t)
    state_estimator.update(12, measurement, r_measurement, uav_state)

if __name__ == "__main__":
    try:
        rospy.init_node('estimate')
        uav_state_pub = rospy.Publisher("/uav_state", Float64MultiArray, queue_size=10)
        # pass all the parameters into the UKF!
        # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
        state_estimator = UKF(12, q, np.zeros(12), 0.0001*np.eye(12), 0.04, 0.0, 2.0, iterate_x,measurement_model)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)
            odom(msg)
            thetac = rospy.wait_for_message('/thetac', Float64MultiArray)
            state = np.array([P1[0],P1[1],0,0,P2[0],P2[1],0,0,P3[0],P3[1],0,0])
            uav_state = np.array([Pc[0],Pc[1],Pc[2],Pr[0],Pr[1],Pr[2],Pb[0],Pb[1],Pb[2],thetac.data[0]])
            measurement = measurement_model(state, uav_state)
            ukf()
            print "Estimated state: ", state_estimator.get_state()
            print "Covariance: ", np.linalg.det(state_estimator.get_covar())
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
