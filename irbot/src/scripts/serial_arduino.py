#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import tf
import time
import serial
import os

pos_x = 0.0
pos_y = 0.0
theta = 0.0

prev_x = 0.0
prev_y = 0.0
prev_theta = 0.0

cmd_vx = 0.0
cmd_wz = 0.0

rpm_ka = None
rpm_ki = None

if __name__ == '__main__':
    try:
        #Init a new node named 'serial_arduino'
        rospy.init_node('irbot_serial_arduino', anonymous=False)

        #Publish to /odom topic
        pub = rospy.Publisher("/odom", Odometry, queue_size=100)

        #serial_port = rospy.get_param("~serial_port","/dev/ttyACM0")
        baud_rate = rospy.get_param("~baud_rate", 57600)

        device_id = rospy.get_param("~device_id",'usb-1a86_USB2.0-Serial-if00-port0')
        serial_port = os.popen('echo "irbot123.." | sudo -S bash {}/get_usb.bash {}'.format(os.path.dirname(os.path.abspath(__file__)), device_id)).read().strip()

        ser = serial.Serial(serial_port, baud_rate, timeout=None)

        rate = rospy.Rate(100) # 100 Hz
        while not rospy.is_shutdown():
            #Subscribe to /cmd_vel topic
            try:
                line_odom = ser.readline().split(",")
                odom_data_parsed = [x.rstrip() for x in line_odom]
            except:
                #time.sleep(0.1)
                serial_port = os.popen('echo "irbot123.." | sudo -S bash {}/get_usb.bash {}'.format(os.path.dirname(os.path.abspath(__file__)), device_id)).read().strip()
                while (serial_port == device_id):
                    time.sleep(0.5)
                    serial_port = os.popen('sudo bash {}/get_usb.bash {}'.format(os.path.dirname(os.path.abspath(__file__)), device_id)).read().strip()
                    print("Searching: ", device_id)

                print("Found the device.")
                ser = serial.Serial(serial_port, baud_rate, timeout=None)

                line_odom = ser.readline().split(",")
                odom_data_parsed = [x.rstrip() for x in line_odom]
                #pass

            #print(line_odom)
            try:
                pos_x = float(odom_data_parsed[0])
                pos_y = float(odom_data_parsed[1])
                theta = float(odom_data_parsed[2])
                rpm_ki = float(odom_data_parsed[3])
                rpm_ka = float(odom_data_parsed[4])
            except:
                pos_x, pos_y, theta = prev_x, prev_y, prev_theta
                #pass

            prev_x = pos_x
            prev_y = pos_y
            prev_theta = theta

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
            print(pos_x, pos_y, theta, cmd_vx, cmd_wz)
            rate.sleep()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

