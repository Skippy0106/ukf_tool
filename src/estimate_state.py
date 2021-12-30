#!/usr/bin/env python

import rospy
from ukf import UKF
import numpy as np
from math import sin,cos,sqrt,atan2
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
import rosbag

time_last = 0
#det_covariance,all_state = Float64MultiArray(),Float64MultiArray()
clock = Clock()

# Process Noise
q = np.eye(6)
q[0][0] = 0.01 
q[1][1] = 0.01
q[2][2] = 0.01
q[3][3] = 0.01
q[4][4] = 0.01
q[5][5] = 0.01


# create measurement noise covariance matrices
r_measurement = np.eye(3)
r_measurement[0][0] = 0.0001
r_measurement[1][1] = 0.0001
r_measurement[2][2] = 0.0001

# create initial matrices
ini = np.array([0,0,0,0,0,0])#[px,py,pz,vx.vy,vz]

def iterate_x(x_in, timestep):
    '''this function is based on the x_dot and can be nonlinear as needed'''
    ret = np.zeros(len(x_in))
    g=9.81
    m=0.3
    ret[0] = x_in[0] + timestep * x_in[3]
    ret[1] = x_in[1] + timestep * x_in[4]
    ret[2] = x_in[2] + timestep * x_in[5]
    ret[3] = x_in[3] 
    ret[4] = x_in[4] 
    ret[5] = x_in[5]
    return ret

def measurement_model(x_in):
    """
    :param x_in: states
    :param data: UAV positions of c,r,b and thetac
    """
    ret = np.zeros(3)
    ret[0] = x_in[0]
    ret[1] = x_in[1]
    ret[2] = x_in[2]
 
    return ret

#def odom(msg):
 #   global Payload_position
  #  payload_index = msg.name.index('payload')
   # Payload_position = np.array([msg.pose[payload_index].position.x, msg.pose[payload_index].position.y, msg.pose[payload_index].position.z])
   
def add_measurementnoise():
    global measurement

    measurement = measurement_model(measurement_state)
    measurement[[0,1,2]] += np.zeros(3)


def ukf():
    global time_last

    d_t = rospy.Time.now().to_sec() - time_last
    state_estimator.predict(d_t)
    state_estimator.update(3, measurement, r_measurement)
    time_last = rospy.Time.now().to_sec()

def clock_cb(msg):
    global clock
    clock = msg


def pose_cb(msg):
    global Payload_position,Payload_vel
    Payload_position=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
    Payload_vel=np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z])

if __name__ == "__main__":
    try:
        rospy.init_node('estimate')
        clock_sub = rospy.Subscriber("/clock", Clock, clock_cb, queue_size=10)
        payload_position_pub = rospy.Publisher("/pose_estimate", Point, queue_size=10)
        position_ground_truth_pub = rospy.Publisher("/pose_ground_truth", Point, queue_size=10)
        vel_ground_truth_pub = rospy.Publisher("/vel_ground_truth", Point, queue_size=10)
        payload_vel_pub = rospy.Publisher("/vel_estimate", Point, queue_size=10)
        Payload_position=np.zeros(3)
        Payload_vel=np.zeros(3)

        # pass all the parameters into the UKF!
        # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
        state_estimator = UKF(6, q, ini, 0.1*np.eye(6), 0.001, 0.0, 2.0, iterate_x,measurement_model)
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            #msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)
            rospy.Subscriber("/payload/position",Odometry,pose_cb)
            #print("payload_position=",Payload_position)
            #thetac_sub = rospy.Subscriber("/theta_iris_camera", Float64MultiArray, theta_update, queue_size=10)
            #while thetac == None:
               # pass
            #state = np.array([0,0,0,0,0,0])
            measurement_state = np.array([Payload_position[0],Payload_position[1],Payload_position[2]])
            vel_ground_truth_pub.publish(Payload_vel[0],Payload_vel[1],Payload_vel[2])
            position_ground_truth_pub.publish(measurement_state[0],measurement_state[1],measurement_state[2])
            add_measurementnoise()
            ukf()
            estimate_state = state_estimator.get_state()
            payload_position_pub.publish(estimate_state[0],estimate_state[1],estimate_state[2])
            payload_vel_pub.publish( estimate_state[3],estimate_state[4],estimate_state[5])
            #state_pub.publish(estimate_state)
#            print "Estimated state: ", state_estimator.get_state()
#            print "Covariance: ", np.linalg.det(state_estimator.get_covar())
           # position_covar = state_estimator.get_covar()
            #position_covar = np.delete(position_covar,[2,3,6,7,10,11],axis=1)
            #position_covar = np.delete(position_covar,[2,3,6,7,10,11],axis=0)
            #det_covariance.data = [np.linalg.det(position_covar),np.linalg.norm([P1[0],P1[1]]-estimate_state[:2]),np.linalg.norm([P2[0],P2[1]]-estimate_state[4:6]),np.linalg.norm([P3[0],P3[1]]-estimate_state[8:10])]
            #print(det_covariance.data)
            #print('--')
            #bag.write('det_covariance', det_covariance)
            #bag.write('/gazebo/model_states', msg)
            #bag.write('/clock', clock)
           # break
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    #finally:
     #   bag.close()
