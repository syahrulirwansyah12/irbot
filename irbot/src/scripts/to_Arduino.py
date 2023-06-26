#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import tf
import serial
import smbus
import struct

bus = smbus.SMBus(1)

arduino_add = 0x68 

pos_x = 0.0
pos_y = 0.0
theta = 0.0

cmd_vx = 0.0
cmd_wz = 0.0

rpm_ka = None
rpm_ki = None

def packData(vx, wz):
    data = struct.pack('ff', vx,wz)
    return [ord(byte) for byte in data]

def cmdVelCallback(vel):
    global cmd_vx, cmd_wz

    cmd_vx = round(vel.linear.x, 2)
    cmd_wz = round(vel.angular.z, 2)

if __name__ == '__main__':
    try:
        #Init a new node named 'serial_arduino'
        rospy.init_node('serial_arduino', anonymous=True)

        #Publish to /odom topic
        pub = rospy.Publisher("/odom",Odometry, queue_size=100)

        serial_port = rospy.get_param("~serial_port","/dev/ttyACM0")
        baud_rate = rospy.get_param("~baud_rate", 57600)

        ser = serial.Serial(serial_port, baud_rate, timeout=None)

        rate = rospy.Rate(100) # 100 Hz
        while not rospy.is_shutdown():
            #Subscribe to /cmd_vel topic
            rospy.Subscriber("/cmd_vel", Twist, cmdVelCallback)

            data = packData(cmd_vx, cmd_wz)
            bus.write_i2c_block_data(arduino_add, 0, data)

            line_odom = ser.readline().split(",")
            odom_data_parsed = [x.rstrip() for x in line_odom]

            print(line_odom)
            try:
                pos_x = float(odom_data_parsed[0])
                pos_y = float(odom_data_parsed[1])
                theta = float(odom_data_parsed[2])
                rpm_ki = float(odom_data_parsed[3])
                rpm_ka = float(odom_data_parsed[4])
            except:
                pos_x, pos_y, theta = 0.0, 0.0, 0.0

            #Create odometry object
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now() 
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"

            odom_msg.pose.pose.position = Point(float(pos_x), float(pos_y), 0.0)
            odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, theta))

            #print(odom_msg)

            pub.publish(odom_msg)
            #rospy.loginfo('Speed = %s m/s, Angular = %s rad/s', cmd_vx, cmd_wz)
            #print(pos_x, pos_y, theta, cmd_vx, cmd_wz)
            rate.sleep()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

