#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import time
from sensor_msgs.msg import LaserScan
import math

field_of_view = 90.0
front_dist = 99.0
right_dist = 99.0
left_dist = 99.0

def move(speed, is_forward):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()
    #get current location 

    if (speed > 0.4):
        print 'speed must be lower than 0.4'
        return

    if (is_forward):
        velocity_message.linear.x =abs(speed)
    else:
       	velocity_message.linear.x =-abs(speed)

    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    velocity_publisher.publish(velocity_message)
    rospy.loginfo("Turtlebot moves forwards")

    loop_rate.sleep()

def stop ():
    velocity_message = Twist()
    velocity_message.linear.x=0
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0

    cmd_vel_topic='/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    velocity_publisher.publish(velocity_message)
    rospy.loginfo("Turtlebot stops")
   
def rotate (angular_speed_degree, clockwise):
    velocity_message = Twist()
    velocity_message.linear.x=0
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0

    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    rospy.loginfo("Turtlesim rotates")
    velocity_publisher.publish(velocity_message)

    loop_rate.sleep()

#find the average within a certain range
def average_between_indices(ranges, i, j):
    slice_of_array = ranges[i: j]
    slice_of_array = [x for x in slice_of_array if not math.isnan(x)]

    i = 0
    for x in slice_of_array:
        if x >= 4.0:
            slice_of_array[i] = 4.0
        i=i+1

def scan_callback(scan_data):
    #Find the front distance
    global front_dist
    global right_dist
    global left_dist

    #scan the 90 degrees side
    min_index_90 = int((90.0-field_of_view/2.0)/(scan_data.angle_increment/3.14*180.0))
    max_index_90 = int((90.0+field_of_view/2.0)/(scan_data.angle_increment/3.14*180.0))
    right_dist = average_between_indices(scan_data.ranges, min_index_90, max_index_90)


    #scan the 270 degrees side
    min_index_270 = int((270.0-field_of_view/2.0)/(scan_data.angle_increment/3.14*180.0))
    max_index_270 = int((270.0+field_of_view/2.0)/(scan_data.angle_increment/3.14*180.0))
    left_dist = average_between_indices(scan_data.ranges, min_index_270, max_index_270)

    #scan the front side
    min_index = int((360.0-field_of_view/2.0)/(scan_data.angle_increment/3.14*180.0)) 
    max_index = int((field_of_view/2.0)/(scan_data.angle_increment/3.14*180.0))
    front_dist = (average_between_indices(scan_data.ranges, min_index, len(scan_data.ranges)) + average_between_indices(scan_data.ranges, 1, max_index))/2.0


if __name__ == '__main__':
    try:
        #init a new node named 'explore_motion'
        rospy.init_node('explore_motion', anonymous=True)

        base_speed = rospy.get_param("~base_speed", 0.2)
        turn = rospy.get_param("~base_turn", 20.0)
        distance_to_stop = rospy.get_param("~distance_stop",0.8)
        distance_to_find = rospy.get_param("~distance_find",3.0)
        field_of_view = rospy.get_param("~field_of_view",30.0) #in deg

        #subscribe to /scan topic
        rospy.Subscriber("scan", LaserScan, scan_callback)

        print "stop distance =", distance_to_stop
        print "find distance =", distance_to_find
        print "field of view =", field_of_view

        loop_rate = rospy.Rate(10)

        while rospy.is_shutdown:
            speed = base_speed + 0.01*front_dist
            move(speed, True)
            print "front distance =", front_dist
            if front_dist < distance_to_stop:
                stop()
                print "front distance =", front_dist
                time.sleep(1.0)
                while front_dist < distance_to_find:
                    rotate(turn, True)
                    print "front distance =", front_dist
                    if front_dist >= distance_to_find:
                        stop()
                        time.sleep(1.0)
                        break
            loop_rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped

    except:
        stop()
    
    finally:
        rospy.loginfo("node terminated.")