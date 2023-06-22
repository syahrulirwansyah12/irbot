#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import serial

pos_x = 0.0
pos_y = 0.0
theta = 0.0

cmd_vx = 0.0
cmd_wz = 0.0

def cmdVelCallback(vel):
    global cmd_vx, cmd_wz
    cmd_vx = round(vel.linear.x, 2)
    cmd_wx = round(vel.angular.z, 2)

def readSerialArduino(port, baud):
    ser = serial.Serial(port, rate, timeout=None)

    line_odom = ser.readline().split(",")
    odom_data_parsed = [x.rstrip() for x in line_odom]

    print(odom_data_parsed)

    pos_x = float(odom_data_parsed[0])
    pos_y = float(odom_data_parsed[1])
    theta = float(odom_data_parsed[2])

def writeSerialArduino(port, baud):
    ser = serial.Serial(port, rate, timeout=None)

    ser.write(b'ROS:')
    ser.write(str(cmd_vx).encode())
    ser.write(b',')
    ser.write(str(cmd_wz).encode())
    ser.write(b'\n')

def pubOdom():
    #Publish to primary /odom topic
    odom_pub = rospy.Publisher("/odom",Odometry,queue_size=10)

    #Create odometry object
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_footprint"
    odom.pose.pose.position.x = pos_x
    odom.pose.pose.position.y = pos_y
    odom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)

    odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        #Init a new node named 'ekf_node'
        rospy.init_node('serial_arduino', anonymous=True)

        #Subscribe to /odom topic
        rospy.Subscriber("/cmd_vel", Twist, cmdVelCallback)

        serial_port = rospy.get_param("~serial_port","/dev/ttyACM0")
        baud_rate = rospy.get_param("~baud_rate", 115200)

        rate = rospy.Rate(100) # 100 Hz
        while not rospy.is_shutdown():
            writeSerialArduino(serial_port, baud_rate)
            readSerialArduino(serial_port, baud_rate)

            pubOdom()
            #rospy.loginfo('Speed = %s m/s, Angular = %s rad/s', cmd_vx, cmd_wz)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
