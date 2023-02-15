#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import time
from sensor_msgs.msg import LaserScan
import math
from std_srvs.srv import Empty

front_dist = 99.0
field_of_view = 99.0

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

def go_to_goal(x_goal, y_goal):
    global x
    global y, z, yaw

    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'

    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print 'x=', x, 'y=',y


        if (distance <0.01):
            break

def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print relative_angle_radians
    print desired_angle_radians
    rotate(30 ,math.degrees(abs(relative_angle_radians)), clockwise)

#find the max range and its index
def min_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return (min(ranges), ranges.index(min(ranges)) )

#find the max range 
def max_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return (max(ranges), ranges.index(max(ranges)) )

#find the average range
def average_range(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return ( sum(ranges) / float(len(ranges)) )

#find the average within a certain range
def average_between_indices(ranges, i, j):
    slice_of_array = ranges[i: j]
    slice_of_array = [x for x in slice_of_array if not math.isnan(x)]

    i = 0
    for x in slice_of_array:
        if x >= 4.0:
            slice_of_array[i] = 4.0
        i=i+1

    return ( sum(slice_of_array) / float(len(slice_of_array)) )

def scan_callback(scan_data):
    #Find the front distance
    global front_dist
    min_index = int((360.0-field_of_view/2.0)/(scan_data.angle_increment/3.14*180.0)) 
    max_index = int((field_of_view/2.0)/(scan_data.angle_increment/3.14*180.0)) 
    front_dist = (average_between_indices(scan_data.ranges, min_index, len(scan_data.ranges)) + average_between_indices(scan_data.ranges, 1, max_index))/2.0


if __name__ == '__main__':
    try:
        #init a new node named 'obs_avoidance_motion'
        rospy.init_node('obs_avoidance_motion', anonymous=True)

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
        