#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf

pos_x = 0.0
pos_y = 0.0
theta = 0.0

def odomCallback(odom_data):
    global pos_x, pos_y, theta
    pos_x = odom_data.pose.pose.position.x*100.0 #in cm
    pos_y = odom_data.pose.pose.position.y*100.0 #in cm

    #print(odom_data.pose.pose.orientation)
    rpy = tf.transformations.euler_from_quaternion([odom_data.pose.pose.orientation.x,
                                                   odom_data.pose.pose.orientation.y,
                                                   odom_data.pose.pose.orientation.z,
                                                   odom_data.pose.pose.orientation.w])
    theta = rpy[2]/3.14*360 #in deg

if __name__ == '__main__':
    try:
        #Init a new node named 'ekf_node'
        rospy.init_node('monitoring_node', anonymous=False)

        print("[Odometry Monitoring]")
        print("Pos_X, Pos_Y, Theta,")

        rate = rospy.Rate(100) # 100 Hz
        while not rospy.is_shutdown():
            #Subscribe to /odom topic
            rospy.Subscriber("/odom", Odometry, odomCallback)
            
            rospy.loginfo('Pos_x: %s, Pos_y: %s, Theta: %s', pos_x, pos_y, theta)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
