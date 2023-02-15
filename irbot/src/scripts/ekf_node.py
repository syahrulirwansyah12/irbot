#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import tf
import math

pose_1 = np.zeros(3) #[x, y, theta]
pose_2 = np.zeros(3)
pose = np.zeros(3)

prev_pose_1 = np.zeros(3) #[x, y, theta]
prev_pose_2 = np.zeros(3)

G = np.identity(3)
H = np.identity(3)

covariance = np.ones((3,3))
R = np.array([1.0, 1.1, 2.2])*np.identity(3, dtype= float) #wheel odometry uncertainties
Q = np.array([1.0, 1.1, 2.2])*np.identity(3, dtype= float) #IMU uncertainties

def odom1Callback(odom_data):
    global pose_1, prev_pose_1, G
    pose_1[0] = odom_data.pose.pose.position.x
    pose_1[1] = odom_data.pose.pose.position.y

    rpy = tf.transformations.euler_from_quaternion(odom_data.pose.pose.quaternion)
    pose_1[2] = rpy[2]

    G[0][2] = (pose_1[0] - prev_pose_1[0])/(pose_1[2] - prev_pose_1[2])
    G[1][2] = (pose_1[1] - prev_pose_1[1])/(pose_1[2] - prev_pose_1[2])

    prev_pose_1 = pose_1

def odom2Callback(odom_data):
    global pose_2, prev_pose_2, H
    pose_2[0] = odom_data.pose.pose.position.x
    pose_2[1] = odom_data.pose.pose.position.y

    rpy = tf.transformations.euler_from_quaternion(odom_data.pose.pose.quaternion)
    pose_2[2] = rpy[2]

    H[0][2] = (pose_2[0] - prev_pose_2[0])/(pose_2[2] - prev_pose_2[2])
    H[1][2] = (pose_2[1] - prev_pose_2[1])/(pose_2[2] - prev_pose_2[2])

    prev_pose_2 = pose_2

def pubOdom():
    #Publish to primary /odom topic
    odom_pub = rospy.Publisher("odom",Odometry,queue_size=10)

    #Create odometry object
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = pose[0]
    odom.pose.pose.position.y = pose[1]
    odom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, pose[2])

    odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        #Init a new node named 'ekf_node'
        rospy.init_node('ekf_node', anonymous=False)

        rate = rospy.Rate(100) # 100 Hz
        while not rospy.is_shutdown():
            #Subscribe to /odom# topic
            rospy.Subscriber("odom1", Odometry, odom1Callback)
            rospy.Subscriber("odom2", Odometry, odom2Callback)

            #Prediction
            pose = pose_1
            covariance = np.matmul(G,
                np.matmul(covariance,
                    np.transpose(G))) + R

            #Correction
            K = np.matmul(covariance,
                np.matmul(np.transpose(H),
                    np.linalg.inv(np.matmul(H, 
                        np.matmul(covariance, 
                            np.transpose(H))) + Q)))
            pose = pose + np.matmul(K, (pose_1 - pose_2))
            covariance = covariance - np.matmul(K, np.matmul(H, covariance))
        
            pubOdom()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

