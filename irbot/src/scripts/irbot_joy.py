#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

axes = []
buttons = []

def joyCallback(joystate):
    global axes, buttons

    axes = joystate.axes
    buttons = joystate.buttons

def pub_twist(max_speed, max_turn):
    global axes, buttons

    twist_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    twist = Twist()    
    try:
        twist.linear.x = (axes[1] + 1.0) * (2.0*max_speed) / 2.0 - max_speed
        if (buttons[4] == 1 and buttons[5] == 0):
            twist.angular.z = max_turn
        elif (buttons[4] == 0 and buttons[5] == 1):
            twist.angular.z = -max_turn
        else:
            twist.angular.z = 0.0

        print("currently:\tspeed %s\tturn %s " % (twist.linear.x, twist.angular.z))

        twist_pub.publish(twist)
    except Exception as e:
        print(e)

def set_vel(linear, angular):
    global axes, buttons

    try:
        if (buttons[-4] == 1):
            linear += 0.001
        elif (buttons[-3] == 1):
            linear -= 0.001

        if (buttons[-1] == 1):
            angular += 0.001
        elif (buttons[-2] == 1):
            angular -= 0.001

        if (buttons[-8] == 1):
            linear = rospy.get_param("~max_speed",0.4)
            angular = rospy.get_param("~max_turn",0.8)

        linear = max(linear, 0)
        angular = max(angular, 0)

    except Exception as e:
        print(e)

    return linear, angular
       

if __name__=="__main__":
    try:
        rospy.init_node('teleop_twist_joystick')

        max_speed = rospy.get_param("~max_speed",0.4)
        max_turn = rospy.get_param("~max_turn",0.8)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            #Subscribe to /odom topic
            rospy.Subscriber("/joy", Joy, joyCallback)

            pub_twist(max_speed, max_turn)
            [max_speed, max_turn] = set_vel(max_speed, max_turn)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
