#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from math import *


class Control:
    def __init__(self, kt, kr, de):
        rospy.init_node("robot_control")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/position", Twist, self.pos_callback)
        rospy.Subscriber("/step", Twist, self.step_callback)
        rospy.Subscriber("/go", Bool, self.go_callback)

        self.rate = rospy.Rate(10)

        self.x = 0
        self.y = 0
        self.theta = 0

        self.kt = kt
        self.kr = kr
        self.de = de

        self.xd     = 0
        self.yd     = 0

        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        self.go = True

    def pos_callback(self, pos):
        self.x = pos.linear.x
        self.y = pos.linear.y
        self.theta = pos.angular.z

    def step_callback(self, goal):
        self.xd = goal.linear.x
        self.yd = goal.linear.y

    def go_callback(self, go):
        self.go = go.data

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)

    def main(self):
        while not rospy.is_shutdown():

            if sqrt((self.xd-self.x)**2 + (self.yd-self.y)**2) >= self.de and self.go:
                rospy.loginfo("going")
                xe = self.x-self.xd
                ye = self.y-self.yd

                thetad = atan2(self.yd-self.y,self.xd-self.x)
                thetae = self.theta - thetad
                
                if abs(thetae) > pi:
                    thetae = (thetae/abs(thetae))*(abs(thetae)-2*pi)
                    
                v = self.kt*sqrt(xe**2+ye**2)
                omega = -self.kr*thetae

                if v > 0.5:
                    v = 0.5
                if omega > pi/8:
                    omega = pi/8
                elif omega < -pi/8:
                    omega = -pi/8

                #v = v*(1-tanh(abs(alpha)*2))

                self.vel.linear.x = v
                self.vel.angular.z = omega

                self.pub.publish(self.vel)

            else:
                self.stop()

            rospy.on_shutdown(self.stop)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Control(0.7, 0.2, 0.01)
        node.main()

    except rospy.ROSInterruptException:
        pass    