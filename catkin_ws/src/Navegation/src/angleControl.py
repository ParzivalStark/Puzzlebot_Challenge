#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import numpy as np
from math import *


class AngleControl:
    def __init__(self, kxp, kxd, kxi, kop):
        rospy.init_node("robot_control")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/position", Twist, self.pos_callback)
        rospy.Subscriber("/theta_e", Float32, self.theta_e_callback)
        rospy.Subscriber("/thetap_e", Float32, self.thetap_e_callback)
        rospy.Subscriber("/x_e", Float32, self.x_e_callback)
        rospy.Subscriber("/xp_e", Float32, self.xp_e_callback)
        rospy.Subscriber("/offset", Float32, self.offset_callback)
        rospy.Subscriber("/go", Bool, self.go_callback)

        self.rate = rospy.Rate(18)

        self.theta = 0

        #self.integral = 0
        self.integral = np.zeros(20) #50

        self.theta_e = 0
        self.thetap_e = 0
        self.x_e = 0
        self.xp_e = 0
        self.offset = 0

        self.kxp = kxp
        self.kxd = kxd #2*sqrt(kxp) #
        self.kxi = kxi
        self.kop = kop

        self.xd     = 0
        self.yd     = 0

        self.vel = Twist()
        self.vel.linear.x = 0.15 #0.38

        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        self.go = True

    def pos_callback(self, pos):
        self.theta = pos.angular.z

    def theta_e_callback(self, error):
        self.theta_e = error.data

    def thetap_e_callback(self, error):
        self.thetap_e = error.data
    
    def x_e_callback(self, error):
        self.x_e = error.data/100
        #self.integral += error.data/100

    def xp_e_callback(self, error):
        self.xp_e = error.data/100

    def offset_callback(self, offset):
        self.offset = offset.data

    def go_callback(self, go):
        self.go = go.data

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)

    def main(self):
        while not rospy.is_shutdown():

            '''
            if self.vel.angular.z > pi/9:
                self.vel.angular.z = pi/9
            elif self.vel.angular.z < -pi/9:
                self.vel.angular.z = -pi/9
                '''

            np.append(self.integral, self.x_e)
            np.delete(self.integral, 0)
            
            self.vel.angular.z = -(self.kxp*self.x_e + self.kxd*self.xp_e + self.kxi*np.sum(self.integral))

            '''
            if self.vel.angular.z > pi/9:
                self.vel.angular.z = pi/9
            elif self.vel.angular.z < -pi/9:
                self.vel.angular.z = -pi/9
                '''

            #self.vel.angular.z = pi/9.7*tanh(self.vel.angular.z)
            self.vel.angular.z = pi/24*tanh(self.vel.angular.z)

            self.pub.publish(self.vel)

            rospy.on_shutdown(self.stop)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        
        #node = AngleControl(0.2, 0.009, 0.006, 0.025) #0.00018, #0.000017
        node = AngleControl(1.0, 0.08, 0.005, 0.025) #kd = 2sqrt(kp)
        
        node.main()

    except rospy.ROSInterruptException:
        pass    