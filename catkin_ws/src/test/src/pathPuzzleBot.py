#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from math import *

global wr
global wl

def stop():
    velocity = Twist()
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0

    pub.publish(velocity)

def wr_callback(velocity):
    global wr
    wr = velocity.data

def wl_callback(velocity):
    global wl 
    wl = velocity.data

if __name__ == '__main__':
    rospy.init_node("Square")
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/wr", Float32, wr_callback)
    rospy.Subscriber("/wl", Float32, wl_callback)

    rate = rospy.Rate(10)

    # Trajectory
    xd = [1.2, 1.2, 0, 0]
    yd = [0, 1.2, 1.2, 0]
    current_goal = 0

    # Wheels' velocities
    wr = 0.0
    wl = 0.0

    # Robot parameters
    r = 0.05
    l = 0.195

    # Starting values
    x = 0.0
    y = 0.0
    angle = 0.0

    # Control values
    kt = 0.7
    kr = 0.2
    de = 0.01

    current_time = rospy.get_time()
    last_time = rospy.get_time()

    velocity = Twist()
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0

    rospy.loginfo('Running...')

    while not rospy.is_shutdown():

        current_time = rospy.get_time()
        dt = current_time - last_time
        last_time = current_time

        if current_goal < len(xd) and len(xd) == len(yd):
            rospy.loginfo('Going to coords ' + str(xd[current_goal]) + ' ' + str(yd[current_goal]))
            if sqrt((xd[current_goal]-x)**2 + (yd[current_goal]-y)**2) >= de:

                rospy.loginfo('Pos: ' + str(x) + ' ' + str(y))

                pub.publish(velocity)

                xe = x-xd[current_goal]
                ye = y-yd[current_goal]

                thetad = atan2(yd[current_goal]-y,xd[current_goal]-x)
                thetae = angle - thetad

                if abs(thetae) > pi:
                    thetae = (thetae/abs(thetae))*(abs(thetae)-2*pi)
                    
                rospy.loginfo('thetae: ' + str(thetae))

                v = kt*sqrt(xe**2+ye**2)
                omega = -kr*thetae

                if v > 0.5:
                    v = 0.5
                if omega > pi/4:
                    omega = pi/4
                elif omega < -pi/4:
                    omega = -pi/4

                v = v*(1-tanh(abs(thetae)*2))

                velocity.linear.x = v
                velocity.angular.z = omega

                distance = r * (wr + wl) * 0.5 * dt
                angle += r * (wr - wl) / l * dt

                if angle < -pi:
                    angle += 2*pi
                elif angle > pi:
                    angle -= 2*pi

                xp = distance*cos(angle)
                yp = distance*sin(angle)

                x = x + xp
                y = y + yp

                rospy.loginfo('xe: ' + str(xe) + ' ye: ' + str(ye))
                rospy.loginfo('Angle: ' + str(angle) + ' thetad: ' + str(thetad))

            else:
                rospy.loginfo('Next goal')
                current_goal += 1

        else:
            rospy.signal_shutdown('Finished')

        rospy.on_shutdown(stop)

        rate.sleep()


