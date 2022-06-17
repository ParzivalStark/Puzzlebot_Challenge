#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from math import pi

global wr
global wl

def stop():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    pub.publish(msg)

def wr_callback(msg):
    global wr
    wr = msg.data

def wl_callback(msg):
    global wl 
    wl = msg.data

if __name__ == '__main__':
    rospy.init_node("Square")
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/wr", Float32, wr_callback)
    rospy.Subscriber("/wl", Float32, wl_callback)

    rate = rospy.Rate(10)

    turn = False
    current_side = 1

    wr = 0.0
    wl = 0.0

    r = 0.05
    l = 0.18
    distance = 0.0
    angle = 0.0
    current_time = rospy.get_time()
    last_time = rospy.get_time()

    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    while not rospy.is_shutdown():

        current_time = rospy.get_time()
        dt = current_time - last_time
        last_time = current_time

        distance += r * (wr + wl) * 0.5 * dt
        angle += r * (wr - wl) / l * dt

        if not turn:
            if distance < 1:
                
                msg.linear.x = 0.2
            elif current_side < 4:
                turn = True
                current_side += 1
                rospy.loginfo("Distance = %f", distance)
                distance = 0
                msg.linear.x = 0.0
            else:
                rospy.signal_shutdown("Finished")
        else:
            if angle < pi/2:
                
                msg.angular.z = 0.1
            else:
                turn = False
                rospy.loginfo("Angle = %f", angle)
                angle = 0
                msg.angular.z = 0.0

        pub.publish(msg)
        rospy.on_shutdown(stop)

        rate.sleep()

    
